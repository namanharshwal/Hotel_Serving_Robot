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

EncoderReader enc_left(PIN_ENC_L_A, PIN_ENC_L_B, false);
EncoderReader enc_right(PIN_ENC_R_A, PIN_ENC_R_B, true);  // Inverted for mirrored axle

VelocityPID pid_left(45.0f, 110.0f, 1.2f, 42.0f);
VelocityPID pid_right(45.0f, 110.0f, 1.2f, 42.0f);

BatteryMonitor battery(PIN_BATT_ADC, 11.0f);
PacketProtocol protocol(Serial);

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
        target_rad_s_left = cmd.left_target_rad_s;
        target_rad_s_right = cmd.right_target_rad_s;
        manual_pwm_mode = false;
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

    // 2. Safety Watchdog Check
    if (now_ms - last_packet_time > WATCHDOG_TIMEOUT_MS) {
        target_rad_s_left = 0.0f;
        target_rad_s_right = 0.0f;
        if (manual_pwm_mode) {
            manual_pwm_l = 0;
            manual_pwm_r = 0;
        }
    }

    // 3. 100 Hz Deterministic Motion Control Loop (every 10,000 us)
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
            // Closed-loop PID velocity control
            if (abs(target_rad_s_left) < 0.02f && abs(actual_w_l) < 0.05f) {
                motor_left.brake();
                pid_left.reset();
            } else {
                float effort_l = pid_left.compute(target_rad_s_left, actual_w_l, dt);
                motor_left.setPwmRaw((int16_t)effort_l);
            }

            if (abs(target_rad_s_right) < 0.02f && abs(actual_w_r) < 0.05f) {
                motor_right.brake();
                pid_right.reset();
            } else {
                float effort_r = pid_right.compute(target_rad_s_right, actual_w_r, dt);
                motor_right.setPwmRaw((int16_t)effort_r);
            }
        }
    }

    // 4. 10 Hz Battery Monitor Sampling
    if (now_ms - last_battery_ms >= 100) {
        last_battery_ms = now_ms;
        battery.update();
    }

    // 5. 50 Hz Telemetry Transmission to Jetson Orin Nano
    if (now_ms - last_telemetry_ms >= 20) {
        last_telemetry_ms = now_ms;

        TelemetryPayload telem;
        telem.left_pos_rad = enc_left.getPositionRad();
        telem.right_pos_rad = enc_right.getPositionRad();
        telem.left_vel_rad_s = enc_left.getVelocityRadS();
        telem.right_vel_rad_s = enc_right.getVelocityRadS();
        telem.battery_voltage = battery.getVoltage();

        telem.status_flags = 0;
        if (estop_active) telem.status_flags |= (1 << 0);
        if (battery.isLowBattery()) telem.status_flags |= (1 << 1);
        if (manual_pwm_mode) telem.status_flags |= (1 << 2);

        protocol.sendTelemetry(telem);
    }

    // 6. Heartbeat LED Blink (every 500ms)
    if (now_ms - last_led_ms >= 500) {
        last_led_ms = now_ms;
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }
}
