/**
 * ==============================================================================
 * VelocityPID.h - Closed-Loop PID + Feedforward Velocity Controller
 * Computes PWM effort for SmartElex 30S motor drivers to match target wheel speed.
 * ==============================================================================
 */

#ifndef VELOCITY_PID_H
#define VELOCITY_PID_H

#include <Arduino.h>

class VelocityPID {
public:
    VelocityPID(float kp = 45.0f, float ki = 110.0f, float kd = 1.2f, float kff = 42.0f);

    // Compute control output [-output_limit, +output_limit]
    float compute(float target_rad_s, float actual_rad_s, float dt);

    void reset();

    // Gain adjustment methods
    void setGains(float kp, float ki, float kd, float kff);
    void setLimits(float output_limit, float integral_limit);

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    float getKff() const { return _kff; }

private:
    float _kp;
    float _ki;
    float _kd;
    float _kff;

    float _integral;
    float _prev_error;
    float _filtered_derivative;

    float _output_limit;
    float _integral_limit;
};

#endif // VELOCITY_PID_H
