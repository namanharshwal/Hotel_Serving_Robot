/**
 * ==============================================================================
 * BatteryMonitor.h - 24V LiFePO4 Battery Monitor for Nammy Robot
 * Reads analog voltage via ADC voltage divider and calculates battery SOC %.
 * ==============================================================================
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
public:
    BatteryMonitor(uint8_t adc_pin, float divider_ratio = 11.0f, float vref = 3.3f);

    void begin();
    void update();

    float getVoltage() const;
    float getPercentage() const;
    bool isLowBattery() const;

private:
    uint8_t _adc_pin;
    float _divider_ratio;
    float _vref;

    float _voltage;
    float _percentage;
};

#endif // BATTERY_MONITOR_H
