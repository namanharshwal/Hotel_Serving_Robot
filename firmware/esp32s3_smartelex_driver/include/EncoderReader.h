/**
 * ==============================================================================
 * EncoderReader.h - Optical Quadrature Encoder Interface for AT120 Otis Motors
 * 
 * Hardware Specifics:
 *   - Motor: 24V AT120 Otis DC Gear Motor (KM949299)
 *   - Motor Encoder: 100 pulses/revolution
 *   - Integrated Gearbox: 15:1 reduction
 *   - 4x Quadrature Decoding: 6,000 counts per wheel revolution
 *   - Wheel Diameter: 100 mm (Radius: 0.050 m)
 *   - Sub-millimeter Resolution: 0.05236 mm / tick
 * ==============================================================================
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
public:
    EncoderReader(uint8_t pin_a, uint8_t pin_b, bool inverted = false);

    void begin();

    // Periodic update called in 100 Hz control loop
    void update(float dt);

    // Tick Count and Position
    int64_t getTicks() const;
    float getPositionRad() const;
    float getDistanceMeters() const;

    // Velocities
    float getVelocityRadS() const;
    float getLinearVelocityMs() const;

    // Reset counters
    void reset();

    // ISR handler hook
    void handleISR();

    static constexpr float TICKS_PER_WHEEL_REV = 6000.0f;
    static constexpr float WHEEL_RADIUS_METERS = 0.050f;
    static constexpr float TWO_PI_F = 6.283185307f;

private:
    uint8_t _pin_a;
    uint8_t _pin_b;
    bool _inverted;
    volatile uint8_t _last_state;

    volatile int64_t _ticks;
    int64_t _prev_ticks;

    float _position_rad;
    float _velocity_rad_s;
    float _distance_m;
    float _linear_vel_m_s;
};

#endif // ENCODER_READER_H
