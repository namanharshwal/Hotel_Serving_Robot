/**
 * ==============================================================================
 * UltrasonicArray.h - Non-Blocking 4-Channel US-100 Ultrasonic Driver
 * Designed for ESP32-S3 mediator on Nammy Robot via TXS0108E Level Shifter.
 * 
 * Hardware:
 *   - 4x US-100 Ultrasonic Distance Sensors (Operating in Trigger/Echo Mode)
 *   - Jumper on back of US-100 MUST be REMOVED (Open) for Trigger/Echo mode!
 *   - 1x TXS0108E 8-Channel High Speed Level Shifter (3.3V <-> 5.0V)
 * 
 * Channels:
 *   1. Front Sonar (Obstacle avoidance & glass detection in forward driving)
 *   2. Left Sonar  (Wall following & lateral clearance)
 *   3. Right Sonar (Wall following & lateral clearance)
 *   4. Rear Sonar  (Obstacle avoidance in reverse driving)
 * ==============================================================================
 */

#ifndef ULTRASONIC_ARRAY_H
#define ULTRASONIC_ARRAY_H

#include <Arduino.h>

#define SONAR_COUNT 4
#define SONAR_MAX_DISTANCE_CM 450 // US-100 max range 4.5m
#define SONAR_MIN_DISTANCE_CM 2   // US-100 min range 2cm
#define SONAR_TIMEOUT_US (SONAR_MAX_DISTANCE_CM * 58) // ~26,100 us

enum SonarIndex {
    SONAR_FRONT = 0,
    SONAR_LEFT  = 1,
    SONAR_RIGHT = 2,
    SONAR_REAR  = 3
};

struct SonarPins {
    uint8_t trig_pin;
    uint8_t echo_pin;
};

class UltrasonicArray {
public:
    UltrasonicArray(const SonarPins pins[SONAR_COUNT]);

    // Initialize GPIO pins
    void begin();

    // Call periodically in main loop (handles non-blocking round-robin pinging)
    void update();

    // Get filtered distance in millimeters (0 if out of range / timeout)
    uint16_t getDistanceMm(SonarIndex index) const;

    // Get filtered distance in centimeters
    float getDistanceCm(SonarIndex index) const;

    // Get raw distance in meters (for ROS 2 Range message)
    float getDistanceMeters(SonarIndex index) const;

    // Check if an obstacle is within critical reflex emergency stop threshold
    bool isFrontObstacleCritical(float threshold_meters = 0.15f) const;
    bool isRearObstacleCritical(float threshold_meters = 0.15f) const;

private:
    SonarPins _pins[SONAR_COUNT];
    uint16_t _distances_mm[SONAR_COUNT];
    
    // Circular buffer for 3-sample median filter
    uint16_t _history[SONAR_COUNT][3];
    uint8_t _history_idx[SONAR_COUNT];

    // Round-robin scheduling state machine
    uint8_t _current_sonar;
    uint32_t _last_ping_us;
    uint32_t _ping_interval_us; // 35ms between pings to prevent acoustic echo crosstalk

    uint16_t computeMedian(uint16_t a, uint16_t b, uint16_t c);
    uint32_t measureEchoDuration(uint8_t trig_pin, uint8_t echo_pin);
};

#endif // ULTRASONIC_ARRAY_H
