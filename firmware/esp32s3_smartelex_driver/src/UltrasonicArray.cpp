#include "UltrasonicArray.h"

UltrasonicArray::UltrasonicArray(const SonarPins pins[SONAR_COUNT])
    : _current_sonar(0), _last_ping_us(0), _ping_interval_us(35000) {
    for (int i = 0; i < SONAR_COUNT; i++) {
        _pins[i] = pins[i];
        _distances_mm[i] = 0;
        _history_idx[i] = 0;
        _history[i][0] = 0;
        _history[i][1] = 0;
        _history[i][2] = 0;
    }
}

void UltrasonicArray::begin() {
    for (int i = 0; i < SONAR_COUNT; i++) {
        pinMode(_pins[i].trig_pin, OUTPUT);
        digitalWrite(_pins[i].trig_pin, LOW);
        pinMode(_pins[i].echo_pin, INPUT);
    }
    _last_ping_us = micros();
}

uint16_t UltrasonicArray::computeMedian(uint16_t a, uint16_t b, uint16_t c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

uint32_t UltrasonicArray::measureEchoDuration(uint8_t trig_pin, uint8_t echo_pin) {
    // Ensure clean LOW pulse before trigger
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    // Send 10us HIGH pulse to trigger US-100
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    // Measure echo pulse width with timeout
    uint32_t duration = pulseIn(echo_pin, HIGH, SONAR_TIMEOUT_US);
    return duration;
}

void UltrasonicArray::update() {
    uint32_t now_us = micros();

    // Check if interval has elapsed for the next sensor in round-robin sequence
    if (now_us - _last_ping_us >= _ping_interval_us) {
        _last_ping_us = now_us;

        uint8_t idx = _current_sonar;
        uint32_t echo_duration = measureEchoDuration(_pins[idx].trig_pin, _pins[idx].echo_pin);

        uint16_t raw_mm = 0;
        if (echo_duration > 0 && echo_duration <= SONAR_TIMEOUT_US) {
            // US-100 distance formula: mm = (duration_us * 343) / 2000
            raw_mm = (uint16_t)((echo_duration * 343ULL) / 2000ULL);
            if (raw_mm < (SONAR_MIN_DISTANCE_CM * 10)) {
                raw_mm = SONAR_MIN_DISTANCE_CM * 10;
            } else if (raw_mm > (SONAR_MAX_DISTANCE_CM * 10)) {
                raw_mm = SONAR_MAX_DISTANCE_CM * 10;
            }
        } else {
            raw_mm = SONAR_MAX_DISTANCE_CM * 10; // Out of range / no obstacle
        }

        // Apply 3-sample median filter to eliminate ultrasonic noise spikes
        uint8_t h_idx = _history_idx[idx];
        _history[idx][h_idx] = raw_mm;
        _history_idx[idx] = (h_idx + 1) % 3;

        _distances_mm[idx] = computeMedian(_history[idx][0], _history[idx][1], _history[idx][2]);

        // Advance to next sonar
        _current_sonar = (_current_sonar + 1) % SONAR_COUNT;
    }
}

uint16_t UltrasonicArray::getDistanceMm(SonarIndex index) const {
    if (index >= 0 && index < SONAR_COUNT) {
        return _distances_mm[index];
    }
    return 0;
}

float UltrasonicArray::getDistanceCm(SonarIndex index) const {
    return getDistanceMm(index) / 10.0f;
}

float UltrasonicArray::getDistanceMeters(SonarIndex index) const {
    return getDistanceMm(index) / 1000.0f;
}

bool UltrasonicArray::isFrontObstacleCritical(float threshold_meters) const {
    float d = getDistanceMeters(SONAR_FRONT);
    return (d > 0.02f && d < threshold_meters);
}

bool UltrasonicArray::isRearObstacleCritical(float threshold_meters) const {
    float d = getDistanceMeters(SONAR_REAR);
    return (d > 0.02f && d < threshold_meters);
}
