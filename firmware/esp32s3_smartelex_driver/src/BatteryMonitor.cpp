#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(uint8_t adc_pin, float divider_ratio, float vref)
    : _adc_pin(adc_pin), _divider_ratio(divider_ratio), _vref(vref),
      _voltage(26.4f), _percentage(0.85f) {}

void BatteryMonitor::begin() {
    analogReadResolution(12);
    pinMode(_adc_pin, INPUT);
    update();
}

void BatteryMonitor::update() {
    uint32_t adc_sum = 0;
    const int SAMPLES = 8;
    for (int i = 0; i < SAMPLES; i++) {
        adc_sum += analogRead(_adc_pin);
        delayMicroseconds(20);
    }
    float raw_adc = (float)adc_sum / SAMPLES;

    // Convert to actual battery voltage
    float pin_voltage = (raw_adc / 4095.0f) * _vref;
    float current_voltage = pin_voltage * _divider_ratio;

    // Exponential moving average filter (alpha = 0.15)
    _voltage = (0.85f * _voltage) + (0.15f * current_voltage);

    // 8S LiFePO4 State-of-Charge (SOC) estimation curve
    if (_voltage >= 28.8f) {
        _percentage = 1.0f;
    } else if (_voltage <= 20.0f) {
        _percentage = 0.0f;
    } else if (_voltage < 25.0f) {
        _percentage = 0.0f + (_voltage - 20.0f) / 5.0f * 0.20f;
    } else if (_voltage < 26.6f) {
        _percentage = 0.20f + (_voltage - 25.0f) / 1.6f * 0.65f;
    } else {
        _percentage = 0.85f + (_voltage - 26.6f) / 2.2f * 0.15f;
    }
    _percentage = constrain(_percentage, 0.0f, 1.0f);
}

float BatteryMonitor::getVoltage() const {
    return _voltage;
}

float BatteryMonitor::getPercentage() const {
    return _percentage;
}

bool BatteryMonitor::isLowBattery() const {
    return (_voltage < 21.6f);  // < 2.70V per cell
}
