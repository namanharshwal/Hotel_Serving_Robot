/**
 * ==============================================================================
 * SmartElex30S.h - Driver Class for Robocraze SmartElex 30S Motor Driver
 * Designed specifically for ESP32-S3 microcontroller on Nammy Robot.
 * 
 * Hardware Features:
 *   - 7V - 28V DC Operating Range (Ideal for 24V LiFePO4 battery pack)
 *   - 30A Continuous Current, 85A Peak
 *   - 3.3V / 5V TTL Logic Compatibility (Direct connection to ESP32-S3 GPIOs)
 *   - High-Frequency Ultrasonic PWM (20 kHz) for silent operation
 *   - Integrated Slew-Rate Limiting & Deadband Friction Compensation
 * ==============================================================================
 */

#ifndef SMARTELEX_30S_H
#define SMARTELEX_30S_H

#include <Arduino.h>

class SmartElex30S {
public:
    SmartElex30S(uint8_t pwm_pin, uint8_t dir_pin, uint8_t ledc_channel, bool inverted = false);

    // Initialize GPIOs and LEDC hardware PWM timer
    void begin(uint32_t freq_hz = 20000, uint8_t res_bits = 10);

    // Set normalized motor effort [-1.0f, +1.0f]
    void setEffort(float effort);

    // Set raw integer duty cycle [-PWM_MAX, +PWM_MAX]
    void setPwmRaw(int16_t duty);

    // Smooth braking / coasting to zero
    void brake();

    // Instantaneous emergency stop
    void emergencyStop();

    // Configuration
    void setInverted(bool inverted);
    void setDeadband(int16_t deadband_duty);
    void setSlewRateLimit(float max_duty_per_sec);

    // Status queries
    int16_t getCurrentDuty() const;
    bool isForward() const;

private:
    uint8_t _pwm_pin;
    uint8_t _dir_pin;
    uint8_t _channel;
    bool _inverted;

    uint32_t _freq;
    uint8_t _resolution;
    int16_t _max_duty;
    int16_t _deadband;

    int16_t _target_duty;
    int16_t _current_duty;
    float _slew_rate; // duty units per second
    uint32_t _last_update_us;

    void writePwm(uint32_t duty);
    void applyOutput(int16_t duty);
};

#endif // SMARTELEX_30S_H
