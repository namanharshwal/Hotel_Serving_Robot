#include "PacketProtocol.h"

uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

PacketProtocol::PacketProtocol(Stream &serial_stream)
    : _stream(serial_stream), _rx_idx(0), _ascii_idx(0),
      _cmd_vel_cb(nullptr), _estop_cb(nullptr), _heartbeat_cb(nullptr),
      _pid_tune_cb(nullptr), _raw_pwm_cb(nullptr) {}

bool PacketProtocol::processIncoming() {
    bool packet_handled = false;

    while (_stream.available()) {
        uint8_t b = _stream.read();

        // 1. Hunt for Header Byte 1 (0xAA)
        if (_rx_idx == 0) {
            if (b == PROTOCOL_HDR1) {
                _rx_buf[0] = b;
                _rx_idx = 1;
            }
            continue;
        }

        // 2. Hunt for Header Byte 2 (0x55)
        if (_rx_idx == 1) {
            if (b == PROTOCOL_HDR2) {
                _rx_buf[1] = b;
                _rx_idx = 2;
            } else if (b == PROTOCOL_HDR1) {
                // Stay at index 1 if consecutive 0xAA received
                _rx_buf[0] = b;
                _rx_idx = 1;
            } else {
                _rx_idx = 0;
            }
            continue;
        }

        // 3. Receive Payload Length, Msg Type, Payload, and CRC16
        _rx_buf[_rx_idx++] = b;

        if (_rx_idx >= 4) {
            uint8_t payload_len = _rx_buf[2];
            size_t total_len = 2 + 1 + 1 + payload_len + 2;

            // Bounds check to avoid buffer overflow
            if (total_len > sizeof(_rx_buf)) {
                _rx_idx = 0;
                continue;
            }

            if (_rx_idx == total_len) {
                // CRC verification
                uint16_t received_crc = _rx_buf[total_len - 2] | (_rx_buf[total_len - 1] << 8);
                uint16_t calc_crc = calculate_crc16(&_rx_buf[2], total_len - 4);

                if (received_crc == calc_crc) {
                    uint8_t msg_type = _rx_buf[3];
                    const uint8_t *payload = &_rx_buf[4];

                    if (msg_type == MSG_CMD_VEL && payload_len == sizeof(CmdVelPayload)) {
                        if (_cmd_vel_cb) {
                            CmdVelPayload cmd;
                            memcpy(&cmd, payload, sizeof(CmdVelPayload));
                            _cmd_vel_cb(cmd);
                        }
                    } else if (msg_type == MSG_ESTOP) {
                        if (_estop_cb) _estop_cb();
                    } else if (msg_type == MSG_HEARTBEAT) {
                        if (_heartbeat_cb) _heartbeat_cb();
                    } else if (msg_type == MSG_PID_TUNE && payload_len == sizeof(PidTunePayload)) {
                        if (_pid_tune_cb) {
                            PidTunePayload tune;
                            memcpy(&tune, payload, sizeof(PidTunePayload));
                            _pid_tune_cb(tune);
                        }
                    } else if (msg_type == MSG_RAW_PWM && payload_len == sizeof(RawPwmPayload)) {
                        if (_raw_pwm_cb) {
                            RawPwmPayload pwm;
                            memcpy(&pwm, payload, sizeof(RawPwmPayload));
                            _raw_pwm_cb(pwm);
                        }
                    }
                    packet_handled = true;
                }
                _rx_idx = 0;
            }
        }
    }

    return packet_handled;
}

void PacketProtocol::handleAsciiCommand(const char *cmd_str) {
    (void)cmd_str;
}

void PacketProtocol::sendTelemetry(const TelemetryPayload &telemetry) {
    uint8_t packet[36];
    packet[0] = PROTOCOL_HDR1;
    packet[1] = PROTOCOL_HDR2;
    packet[2] = 30;               // Payload length (22 + 8 bytes sonars)
    packet[3] = MSG_TELEMETRY;

    memcpy(&packet[4], &telemetry.left_pos_rad, 4);
    memcpy(&packet[8], &telemetry.right_pos_rad, 4);
    memcpy(&packet[12], &telemetry.left_vel_rad_s, 4);
    memcpy(&packet[16], &telemetry.right_vel_rad_s, 4);
    memcpy(&packet[20], &telemetry.battery_voltage, 4);
    memcpy(&packet[24], &telemetry.status_flags, 2);
    memcpy(&packet[26], &telemetry.sonar_front_mm, 2);
    memcpy(&packet[28], &telemetry.sonar_left_mm, 2);
    memcpy(&packet[30], &telemetry.sonar_right_mm, 2);
    memcpy(&packet[32], &telemetry.sonar_rear_mm, 2);

    uint16_t crc = calculate_crc16(&packet[2], 32);
    packet[34] = crc & 0xFF;
    packet[35] = (crc >> 8) & 0xFF;

    _stream.write(packet, 36);
}
