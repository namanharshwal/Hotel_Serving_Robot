/**
 * ==============================================================================
 * PacketProtocol.h - Binary Framed Serial Protocol with CRC16
 * Connects ESP32-S3 mediator with NVIDIA Jetson Orin Nano.
 * ==============================================================================
 */

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <Arduino.h>

#define PROTOCOL_HDR1     0xAA
#define PROTOCOL_HDR2     0x55

// Message IDs
#define MSG_CMD_VEL       0x01
#define MSG_TELEMETRY     0x02
#define MSG_HEARTBEAT     0x03
#define MSG_ESTOP         0x04
#define MSG_PID_TUNE      0x05
#define MSG_RAW_PWM       0x06

struct CmdVelPayload {
    float left_target_rad_s;
    float right_target_rad_s;
};

struct TelemetryPayload {
    float left_pos_rad;
    float right_pos_rad;
    float left_vel_rad_s;
    float right_vel_rad_s;
    float battery_voltage;
    uint16_t status_flags;
    uint16_t sonar_front_mm;
    uint16_t sonar_left_mm;
    uint16_t sonar_right_mm;
    uint16_t sonar_rear_mm;
};

struct PidTunePayload {
    float kp;
    float ki;
    float kd;
    float kff;
};

struct RawPwmPayload {
    int16_t left_duty;
    int16_t right_duty;
};

uint16_t calculate_crc16(const uint8_t *data, size_t len);

class PacketProtocol {
public:
    PacketProtocol(Stream &serial_stream);

    // Call in loop to process incoming serial bytes
    bool processIncoming();

    // Send Telemetry packet to Jetson
    void sendTelemetry(const TelemetryPayload &telemetry);

    // Callbacks
    typedef void (*CmdVelCallback)(const CmdVelPayload &cmd);
    typedef void (*EstopCallback)();
    typedef void (*HeartbeatCallback)();
    typedef void (*PidTuneCallback)(const PidTunePayload &tune);
    typedef void (*RawPwmCallback)(const RawPwmPayload &pwm);

    void onCmdVel(CmdVelCallback cb) { _cmd_vel_cb = cb; }
    void onEstop(EstopCallback cb) { _estop_cb = cb; }
    void onHeartbeat(HeartbeatCallback cb) { _heartbeat_cb = cb; }
    void onPidTune(PidTuneCallback cb) { _pid_tune_cb = cb; }
    void onRawPwm(RawPwmCallback cb) { _raw_pwm_cb = cb; }

private:
    Stream &_stream;
    uint8_t _rx_buf[64];
    size_t _rx_idx;

    char _ascii_buf[48];
    size_t _ascii_idx;

    CmdVelCallback _cmd_vel_cb;
    EstopCallback _estop_cb;
    HeartbeatCallback _heartbeat_cb;
    PidTuneCallback _pid_tune_cb;
    RawPwmCallback _raw_pwm_cb;

    void handleAsciiCommand(const char *cmd_str);
};

#endif // PACKET_PROTOCOL_H
