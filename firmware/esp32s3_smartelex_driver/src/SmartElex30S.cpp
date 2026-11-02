#include "SmartElex30S.h"
#include <esp_arduino_version.h>

SmartElex30S::SmartElex30S(uint8_t pwm_pin, uint8_t dir_pin, uint8_t ledc_channel, bool inverted)
    : _pwm_pin(pwm_pin), _dir_pin(dir_pin), _channel(ledc_channel), _inverted(inverted),
      _freq(20000), _resolution(10), _max_duty(1023), _deadband(60),
      _target_duty(0), _current_duty(0), _slew_rate(4000.0f), _last_update_us(0) {}

void SmartElex30S::begin(uint32_t freq_hz, uint8_t res_bits) {
    _freq = freq_hz;
    _resolution = res_bits;
    _max_duty = (1 << _resolution) - 1;

    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_dir_pin, LOW);

    // Setup ESP32 LEDC PWM peripheral (compatible with Arduino ESP32 core 2.x and 3.x)
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttachChannel(_pwm_pin, _freq, _resolution, _channel);
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

    // Apply slew rate limiting for smooth high-torque transition
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
    if (abs(duty) <= 5) {
        writePwm(0);
        return;
    }

    // Deadband friction compensation
    int16_t effective_duty = duty;
    if (effective_duty > 0) {
        effective_duty = map(effective_duty, 1, _max_duty, _deadband, _max_duty);
    } else {
        effective_duty = -map(-effective_duty, 1, _max_duty, _deadband, _max_duty);
    }

    bool forward = (effective_duty >= 0);
    if (_inverted) {
        forward = !forward;
    }

    uint32_t pwm_val = (uint32_t)abs(effective_duty);
    pwm_val = min((uint32_t)_max_duty, pwm_val);

    // SmartElex 30S Pin Logic:
    // DIR HIGH = Forward spin
    // DIR LOW  = Reverse spin
    // PWM      = Motor speed
    digitalWrite(_dir_pin, forward ? HIGH : LOW);
    writePwm(pwm_val);
}

void SmartElex30S::brake() {
    _target_duty = 0;
    _current_duty = 0;
    writePwm(0);
}

void SmartElex30S::emergencyStop() {
    _target_duty = 0;
    _current_duty = 0;
    writePwm(0);
    digitalWrite(_dir_pin, LOW);
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
