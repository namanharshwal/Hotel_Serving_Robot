#include "VelocityPID.h"

VelocityPID::VelocityPID(float kp, float ki, float kd, float kff)
    : _kp(kp), _ki(ki), _kd(kd), _kff(kff),
      _integral(0.0f), _prev_error(0.0f), _filtered_derivative(0.0f),
      _output_limit(1023.0f), _integral_limit(300.0f) {}

float VelocityPID::compute(float target_rad_s, float actual_rad_s, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // If target is zero, reset integral and return zero
    if (abs(target_rad_s) < 0.01f && abs(actual_rad_s) < 0.05f) {
        reset();
        return 0.0f;
    }

    float error = target_rad_s - actual_rad_s;

    // Feedforward term provides baseline duty cycle proportional to desired speed
    float feedforward = _kff * target_rad_s;

    // Proportional term
    float p_out = _kp * error;

    // Integral term with anti-windup clamping
    _integral += error * dt;
    _integral = constrain(_integral, -_integral_limit, _integral_limit);
    float i_out = _ki * _integral;

    // Filtered derivative term
    float raw_derivative = (error - _prev_error) / dt;
    _prev_error = error;
    _filtered_derivative = (0.7f * _filtered_derivative) + (0.3f * raw_derivative);
    float d_out = _kd * _filtered_derivative;

    // Combined output
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
