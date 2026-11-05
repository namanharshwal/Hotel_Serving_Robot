/**
 * ==============================================================================
 * main.cpp - ESP32-S3 Real-Time Wheel Mediator for Nammy Robot
 * Interfaces NVIDIA Jetson Orin Nano with 2x SmartElex 30S and AT120 DC Motors.
 * ==============================================================================
 */

#include <Arduino.h>
#include "SmartElex30S.h"
#include "EncoderReader.h"
#include "VelocityPID.h"
#include "BatteryMonitor.h"
#include "PacketProtocol.h"
#include "UltrasonicArray.h"

// ==============================================================================
// HARDWARE PIN ASSIGNMENTS
// ==============================================================================
// SmartElex 30S Left Motor Driver
#define PIN_MOTOR_L_PWM       4
#define PIN_MOTOR_L_DIR       5

// SmartElex 30S Right Motor Driver
#define PIN_MOTOR_R_PWM       6
#define PIN_MOTOR_R_DIR       7

// AT120 Optical Encoders (100 pulses/rev * 15:1 gear ratio = 6000 ticks/wheel rev)
#define PIN_ENC_L_A           15
#define PIN_ENC_L_B           16
#define PIN_ENC_R_A           17
#define PIN_ENC_R_B           18

// 4x US-100 Ultrasonic Sensors via TXS0108E Level Shifter (8 Channels A1..A8)
#define PIN_SONAR_FRONT_TRIG  8
#define PIN_SONAR_FRONT_ECHO  9
#define PIN_SONAR_LEFT_TRIG   10
#define PIN_SONAR_LEFT_ECHO   11
#define PIN_SONAR_RIGHT_TRIG  12
#define PIN_SONAR_RIGHT_ECHO  13
#define PIN_SONAR_REAR_TRIG   14
#define PIN_SONAR_REAR_ECHO   21

// 24V LiFePO4 Battery Voltage Divider
#define PIN_BATT_ADC          1

// Status LED (Blinks on heartbeat)
#define PIN_STATUS_LED        2

#define WATCHDOG_TIMEOUT_MS   300

// ==============================================================================
// GLOBAL HARDWARE DRIVER INSTANCES
// ==============================================================================
SmartElex30S motor_left(PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR, 0, false);
SmartElex30S motor_right(PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR, 1, false);

EncoderReader enc_left(PIN_ENC_L_A, PIN_ENC_L_B, true);
EncoderReader enc_right(PIN_ENC_R_A, PIN_ENC_R_B, true);  // Inverted for mirrored axle

VelocityPID pid_left(45.0f, 110.0f, 1.2f, 42.0f);
VelocityPID pid_right(45.0f, 110.0f, 1.2f, 42.0f);

BatteryMonitor battery(PIN_BATT_ADC, 11.0f);
PacketProtocol protocol(Serial);

const SonarPins sonar_pins[SONAR_COUNT] = {
    {PIN_SONAR_FRONT_TRIG, PIN_SONAR_FRONT_ECHO},
    {PIN_SONAR_LEFT_TRIG,  PIN_SONAR_LEFT_ECHO},
    {PIN_SONAR_RIGHT_TRIG, PIN_SONAR_RIGHT_ECHO},
    {PIN_SONAR_REAR_TRIG,  PIN_SONAR_REAR_ECHO}
};
UltrasonicArray sonar(sonar_pins);

// Motion Targets and Flags
float target_rad_s_left = 0.0f;
float target_rad_s_right = 0.0f;
bool estop_active = false;
bool manual_pwm_mode = false;
int16_t manual_pwm_l = 0;
int16_t manual_pwm_r = 0;
uint32_t last_packet_time = 0;

// ==============================================================================
// INTERRUPT SERVICE ROUTINES
// ==============================================================================
void IRAM_ATTR isr_left_encoder() {
    enc_left.handleISR();
}

void IRAM_ATTR isr_right_encoder() {
    enc_right.handleISR();
}

// ==============================================================================
// PROTOCOL CALLBACKS (FROM JETSON ORIN NANO)
// ==============================================================================
void on_cmd_vel(const CmdVelPayload &cmd) {
    if (!estop_active) {
        // Defensive bounds & NaN/Inf verification
        if (!isnan(cmd.left_target_rad_s) && !isinf(cmd.left_target_rad_s) &&
            !isnan(cmd.right_target_rad_s) && !isinf(cmd.right_target_rad_s)) {
            // Strictly clamp setpoint within rated motor physical envelope (+/- 25 rad/s)
            target_rad_s_left = constrain(cmd.left_target_rad_s, -25.0f, 25.0f);
            target_rad_s_right = constrain(cmd.right_target_rad_s, -25.0f, 25.0f);
            manual_pwm_mode = false;
        } else {
            target_rad_s_left = 0.0f;
            target_rad_s_right = 0.0f;
        }
    }
    last_packet_time = millis();
}

void on_estop() {
    estop_active = !estop_active;
    if (estop_active) {
        target_rad_s_left = 0.0f;
        target_rad_s_right = 0.0f;
        motor_left.emergencyStop();
        motor_right.emergencyStop();
        pid_left.reset();
        pid_right.reset();
    }
    last_packet_time = millis();
}

void on_heartbeat() {
    last_packet_time = millis();
}

void on_pid_tune(const PidTunePayload &tune) {
    pid_left.setGains(tune.kp, tune.ki, tune.kd, tune.kff);
    pid_right.setGains(tune.kp, tune.ki, tune.kd, tune.kff);
    last_packet_time = millis();
}

void on_raw_pwm(const RawPwmPayload &pwm) {
    if (!estop_active) {
        manual_pwm_mode = true;
        manual_pwm_l = pwm.left_duty;
        manual_pwm_r = pwm.right_duty;
    }
    last_packet_time = millis();
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
    Serial.begin(115200);

    // Initialize Motor Drivers at 20 kHz ultrasonic frequency
    motor_left.begin(20000, 10);
    motor_right.begin(20000, 10);

    // Initialize Encoders
    enc_left.begin();
    enc_right.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isr_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_right_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isr_right_encoder, CHANGE);

    // Initialize Battery ADC
    battery.begin();

    // Initialize 4-Channel US-100 Ultrasonic Array
    sonar.begin();

    // Register Callbacks
    protocol.onCmdVel(on_cmd_vel);
    protocol.onEstop(on_estop);
    protocol.onHeartbeat(on_heartbeat);
    protocol.onPidTune(on_pid_tune);
    protocol.onRawPwm(on_raw_pwm);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    last_packet_time = millis();
}

// ==============================================================================
// MAIN MEDIATOR LOOP
// ==============================================================================
void loop() {
    static uint32_t last_control_us = 0;
    static uint32_t last_telemetry_ms = 0;
    static uint32_t last_battery_ms = 0;
    static uint32_t last_led_ms = 0;

    uint32_t now_us = micros();
    uint32_t now_ms = millis();

    // 1. Process Serial Packets from Jetson
    protocol.processIncoming();

    // 2. Non-blocking round-robin pinging of 4x US-100 Sonars
    sonar.update();

    // 3. Safety Watchdog Check
    if (now_ms - last_packet_time > WATCHDOG_TIMEOUT_MS) {
        target_rad_s_left = 0.0f;
        target_rad_s_right = 0.0f;
        if (manual_pwm_mode) {
            manual_pwm_l = 0;
            manual_pwm_r = 0;
        }
    }

    // Check Reflex Obstacle Detection (Critical safety distance < 18cm)
    bool front_obstacle = sonar.isFrontObstacleCritical(0.18f);
    bool rear_obstacle = sonar.isRearObstacleCritical(0.18f);

    // 4. 100 Hz Deterministic Motion Control Loop (every 10,000 us)
    if (now_us - last_control_us >= 10000) {
        float dt = (now_us - last_control_us) * 1e-6f;
        last_control_us = now_us;

        // Update wheel encoders
        enc_left.update(dt);
        enc_right.update(dt);

        float actual_w_l = enc_left.getVelocityRadS();
        float actual_w_r = enc_right.getVelocityRadS();

        if (estop_active) {
            motor_left.emergencyStop();
            motor_right.emergencyStop();
        } else if (manual_pwm_mode) {
            motor_left.setPwmRaw(manual_pwm_l);
            motor_right.setPwmRaw(manual_pwm_r);
        } else {
            // Apply Reflex Obstacle Protection
            float eff_target_l = target_rad_s_left;
            float eff_target_r = target_rad_s_right;

            // Reflex: block forward motion if obstacle detected directly in front
            if (front_obstacle && (eff_target_l > 0.0f || eff_target_r > 0.0f)) {
                eff_target_l = min(0.0f, eff_target_l);
                eff_target_r = min(0.0f, eff_target_r);
            }
            // Reflex: block backward motion if obstacle detected directly behind
            if (rear_obstacle && (eff_target_l < 0.0f || eff_target_r < 0.0f)) {
                eff_target_l = max(0.0f, eff_target_l);
                eff_target_r = max(0.0f, eff_target_r);
            }

            // Closed-loop PID velocity control
            if (abs(eff_target_l) < 0.02f && abs(actual_w_l) < 0.05f) {
                motor_left.brake();
                pid_left.reset();
            } else {
                float effort_l = pid_left.compute(eff_target_l, actual_w_l, dt);
                motor_left.setPwmRaw((int16_t)effort_l);
            }

            if (abs(eff_target_r) < 0.02f && abs(actual_w_r) < 0.05f) {
                motor_right.brake();
                pid_right.reset();
            } else {
                float effort_r = pid_right.compute(eff_target_r, actual_w_r, dt);
                motor_right.setPwmRaw((int16_t)effort_r);
            }
        }
    }

    // 5. 10 Hz Battery Monitor Sampling
    if (now_ms - last_battery_ms >= 100) {
        last_battery_ms = now_ms;
        battery.update();
    }

    // 6. 50 Hz Telemetry Transmission to Jetson Orin Nano
    if (now_ms - last_telemetry_ms >= 20) {
        last_telemetry_ms = now_ms;

        TelemetryPayload telem;
        telem.left_pos_rad = enc_left.getPositionRad();
        telem.right_pos_rad = enc_right.getPositionRad();
        telem.left_vel_rad_s = enc_left.getVelocityRadS();
        telem.right_vel_rad_s = enc_right.getVelocityRadS();
        telem.battery_voltage = battery.getVoltage();
        telem.sonar_front_mm = sonar.getDistanceMm(SONAR_FRONT);
        telem.sonar_left_mm  = sonar.getDistanceMm(SONAR_LEFT);
        telem.sonar_right_mm = sonar.getDistanceMm(SONAR_RIGHT);
        telem.sonar_rear_mm  = sonar.getDistanceMm(SONAR_REAR);

        telem.status_flags = 0;
        if (estop_active) telem.status_flags |= (1 << 0);
        if (battery.isLowBattery()) telem.status_flags |= (1 << 1);
        if (manual_pwm_mode) telem.status_flags |= (1 << 2);
        if (front_obstacle) telem.status_flags |= (1 << 3);
        if (rear_obstacle) telem.status_flags |= (1 << 4);

        protocol.sendTelemetry(telem);
    }

    // 7. Heartbeat LED Blink (every 500ms)
    if (now_ms - last_led_ms >= 500) {
        last_led_ms = now_ms;
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }
}
