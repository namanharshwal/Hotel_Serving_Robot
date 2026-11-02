#include "EncoderReader.h"

EncoderReader::EncoderReader(uint8_t pin_a, uint8_t pin_b, bool inverted)
    : _pin_a(pin_a), _pin_b(pin_b), _inverted(inverted),
      _ticks(0), _prev_ticks(0),
      _position_rad(0.0f), _velocity_rad_s(0.0f),
      _distance_m(0.0f), _linear_vel_m_s(0.0f) {}

void EncoderReader::begin() {
    pinMode(_pin_a, INPUT_PULLUP);
    pinMode(_pin_b, INPUT_PULLUP);
    reset();
}

void IRAM_ATTR EncoderReader::handleISR() {
    bool a = digitalRead(_pin_a);
    bool b = digitalRead(_pin_b);

    int8_t dir = (a == b) ? 1 : -1;
    if (_inverted) {
        dir = -dir;
    }
    _ticks += dir;
}

void EncoderReader::update(float dt) {
    if (dt <= 0.0f) return;

    // Atomically sample volatile ticks
    noInterrupts();
    int64_t current_ticks = _ticks;
    interrupts();

    int64_t delta_ticks = current_ticks - _prev_ticks;
    _prev_ticks = current_ticks;

    // Angle and distance
    _position_rad = ((float)current_ticks / TICKS_PER_WHEEL_REV) * TWO_PI_F;
    _distance_m = _position_rad * WHEEL_RADIUS_METERS;

    // Instantaneous angular velocity (rad/s)
    float raw_vel_rad_s = ((float)delta_ticks / TICKS_PER_WHEEL_REV) * TWO_PI_F / dt;

    // 1st-order Low-Pass Filter (alpha = 0.4) to eliminate quantization jitter
    _velocity_rad_s = (0.6f * _velocity_rad_s) + (0.4f * raw_vel_rad_s);
    _linear_vel_m_s = _velocity_rad_s * WHEEL_RADIUS_METERS;
}

int64_t EncoderReader::getTicks() const {
    return _ticks;
}

float EncoderReader::getPositionRad() const {
    return _position_rad;
}

float EncoderReader::getDistanceMeters() const {
    return _distance_m;
}

float EncoderReader::getVelocityRadS() const {
    return _velocity_rad_s;
}

float EncoderReader::getLinearVelocityMs() const {
    return _linear_vel_m_s;
}

void EncoderReader::reset() {
    noInterrupts();
    _ticks = 0;
    _prev_ticks = 0;
    interrupts();
    _position_rad = 0.0f;
    _velocity_rad_s = 0.0f;
    _distance_m = 0.0f;
    _linear_vel_m_s = 0.0f;
}
