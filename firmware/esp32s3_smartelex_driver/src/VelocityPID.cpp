#include "VelocityPID.h"

VelocityPID::VelocityPID(float kp, float ki, float kd, float kff)
    : _kp(kp), _ki(ki), _kd(kd), _kff(kff),
      _integral(0.0f), _prev_error(0.0f), _filtered_derivative(0.0f),
      _output_limit(1023.0f), _integral_limit(150.0f) {}

float VelocityPID::compute(float target_rad_s, float actual_rad_s, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // If target is zero and motor is nearly stopped, reset and return zero
    if (abs(target_rad_s) < 0.01f && abs(actual_rad_s) < 0.05f) {
        reset();
        return 0.0f;
    }

    float error = target_rad_s - actual_rad_s;

    // Feedforward term provides baseline duty cycle proportional to desired speed
    float feedforward = _kff * target_rad_s;

    // Proportional term
    float p_out = _kp * error;

    // Integral term with anti-windup clamping (integral contribution capped at _integral_limit)
    _integral += error * dt;
    float max_int = (_ki > 0.001f) ? (_integral_limit / _ki) : 0.0f;
    _integral = constrain(_integral, -max_int, max_int);
    float i_out = _ki * _integral;

    // Filtered derivative term (only active if _kd > 0)
    float d_out = 0.0f;
    if (_kd > 0.001f) {
        float raw_derivative = (error - _prev_error) / dt;
        _prev_error = error;
        _filtered_derivative = (0.7f * _filtered_derivative) + (0.3f * raw_derivative);
        d_out = _kd * _filtered_derivative;
    } else {
        _prev_error = error;
    }

    // Combined output clamped to hardware limits
    float total_effort = feedforward + p_out + i_out + d_out;
    return constrain(total_effort, -_output_limit, _output_limit);
}

void VelocityPID::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
    _filtered_derivative = 0.0f;
}

void VelocityPID::setGains(float kp, float ki, float kd, float kff) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _kff = kff;
    reset();
}

void VelocityPID::setLimits(float output_limit, float integral_limit) {
    _output_limit = output_limit;
    _integral_limit = integral_limit;
}
