#include "SmartElex30S.h"
#include <esp_arduino_version.h>

SmartElex30S::SmartElex30S(uint8_t pwm_pin, uint8_t dir_pin, uint8_t ledc_channel, bool inverted)
    : _pwm_pin(pwm_pin), _dir_pin(dir_pin), _channel(ledc_channel), _inverted(inverted),
      _freq(490), _resolution(10), _max_duty(1023), _deadband(25),
      _target_duty(0), _current_duty(0), _slew_rate(800.0f), _last_update_us(0) {}

void SmartElex30S::begin(uint32_t freq_hz, uint8_t res_bits) {
    _freq = freq_hz;
    _resolution = res_bits;
    _max_duty = (1 << _resolution) - 1;

    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_dir_pin, LOW);
    pinMode(_pwm_pin, OUTPUT);
    digitalWrite(_pwm_pin, LOW);

    // Setup ESP32 LEDC PWM peripheral (compatible with Arduino ESP32 core 2.x and 3.x)
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(_pwm_pin, _freq, _resolution);
#else
    ledcSetup(_channel, _freq, _resolution);
    ledcAttachPin(_pwm_pin, _channel);
#endif
    writePwm(0);

    _last_update_us = micros();
}

void SmartElex30S::writePwm(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(_pwm_pin, duty);
#else
    ledcWrite(_channel, duty);
#endif
}

void SmartElex30S::setEffort(float effort) {
    effort = constrain(effort, -1.0f, 1.0f);
    setPwmRaw((int16_t)(effort * _max_duty));
}

void SmartElex30S::setPwmRaw(int16_t duty) {
    _target_duty = constrain(duty, -_max_duty, _max_duty);

    // Apply slew rate limiting for smooth torque transition
    uint32_t now_us = micros();
    float dt = (now_us - _last_update_us) * 1e-6f;
    _last_update_us = now_us;

    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    float max_delta = _slew_rate * dt;
    float delta = _target_duty - _current_duty;

    if (delta > max_delta) {
        _current_duty += (int16_t)max_delta;
    } else if (delta < -max_delta) {
        _current_duty -= (int16_t)max_delta;
    } else {
        _current_duty = _target_duty;
    }

    applyOutput(_current_duty);
}

void SmartElex30S::applyOutput(int16_t duty) {
    if (duty == 0) {
        writePwm(0);
        return;
    }

    bool forward = (duty >= 0);
    if (_inverted) {
        forward = !forward;
    }

    uint32_t duty_mag = (uint32_t)abs(duty);
    
    // Smooth deadband blending: smoothly add static friction offset as duty increases
    // Eliminates discontinuous step jumps between 0 and deadband
    float blend = min(1.0f, (float)duty_mag / 80.0f);
    uint32_t effective_duty = duty_mag + (uint32_t)(_deadband * blend);
    effective_duty = min((uint32_t)_max_duty, effective_duty);

    // SmartElex 30S Pin Logic:
    // DIR HIGH = Forward spin
    // DIR LOW  = Reverse spin
    // PWM      = Motor speed
    digitalWrite(_dir_pin, forward ? HIGH : LOW);
    writePwm(effective_duty);
}

void SmartElex30S::brake() {
    _target_duty = 0;
    _current_duty = 0;
    writePwm(0);
}

void SmartElex30S::emergencyStop() {
    brake();
}

void SmartElex30S::setInverted(bool inverted) {
    _inverted = inverted;
}

void SmartElex30S::setDeadband(int16_t deadband_duty) {
    _deadband = deadband_duty;
}

void SmartElex30S::setSlewRateLimit(float max_duty_per_sec) {
    _slew_rate = max_duty_per_sec;
}

int16_t SmartElex30S::getCurrentDuty() const {
    return _current_duty;
}

bool SmartElex30S::isForward() const {
    return (_current_duty >= 0);
}
