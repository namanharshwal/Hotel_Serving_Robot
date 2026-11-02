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

        // 1. Check for human-readable ASCII commands (e.g. starting with 'm', 'p', 's', 't', '?')
        if (_rx_idx == 0 && (b >= 32 && b <= 126)) {
            if (b == '\r' || b == '\n') {
                if (_ascii_idx > 0) {
                    _ascii_buf[_ascii_idx] = '\0';
                    handleAsciiCommand(_ascii_buf);
                    _ascii_idx = 0;
                    packet_handled = true;
                }
            } else if (_ascii_idx < sizeof(_ascii_buf) - 1) {
                _ascii_buf[_ascii_idx++] = (char)b;
            }
            continue;
        }

        // 2. Binary Framed Packet State Machine
        if (_rx_idx == 0 && b != PROTOCOL_HDR1) {
            continue;
        }
        if (_rx_idx == 1 && b != PROTOCOL_HDR2) {
            _rx_idx = 0;
            continue;
        }

        _rx_buf[_rx_idx++] = b;

        if (_rx_idx >= 6) {
            uint8_t payload_len = _rx_buf[2];
            size_t total_len = 2 + 1 + 1 + payload_len + 2;

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

        if (_rx_idx >= sizeof(_rx_buf)) {
            _rx_idx = 0;
        }
    }

    return packet_handled;
}

void PacketProtocol::handleAsciiCommand(const char *cmd_str) {
    if (cmd_str[0] == 'm') { // m <left_rad_s> <right_rad_s>
        float l = 0.0f, r = 0.0f;
        if (sscanf(cmd_str + 1, "%f %f", &l, &r) >= 1) {
            if (_cmd_vel_cb) {
                CmdVelPayload cmd = {l, r};
                _cmd_vel_cb(cmd);
            }
            _stream.printf("[OK] Set wheel speeds: L=%.2f, R=%.2f rad/s\n", l, r);
        }
    } else if (cmd_str[0] == 'p') { // p <left_pwm> <right_pwm>
        int l_pwm = 0, r_pwm = 0;
        if (sscanf(cmd_str + 1, "%d %d", &l_pwm, &r_pwm) >= 1) {
            if (_raw_pwm_cb) {
                RawPwmPayload pwm = {(int16_t)l_pwm, (int16_t)r_pwm};
                _raw_pwm_cb(pwm);
            }
            _stream.printf("[OK] Set raw PWM: L=%d, R=%d\n", l_pwm, r_pwm);
        }
    } else if (cmd_str[0] == 's') { // Stop
        if (_estop_cb) _estop_cb();
        _stream.println("[OK] Emergency Stop triggered.");
    } else if (cmd_str[0] == '?') {
        _stream.println("=== Nammy Robot ESP32-S3 Mediator CLI ===");
        _stream.println("  m <l> <r>  : Set wheel speeds (rad/s)");
        _stream.println("  p <l> <r>  : Set raw PWM (-1023 to 1023)");
        _stream.println("  s          : Emergency Stop");
        _stream.println("==========================================");
    }
}

void PacketProtocol::sendTelemetry(const TelemetryPayload &telemetry) {
    uint8_t packet[28];
    packet[0] = PROTOCOL_HDR1;
    packet[1] = PROTOCOL_HDR2;
    packet[2] = 22;               // Payload length
    packet[3] = MSG_TELEMETRY;

    memcpy(&packet[4], &telemetry.left_pos_rad, 4);
    memcpy(&packet[8], &telemetry.right_pos_rad, 4);
    memcpy(&packet[12], &telemetry.left_vel_rad_s, 4);
    memcpy(&packet[16], &telemetry.right_vel_rad_s, 4);
    memcpy(&packet[20], &telemetry.battery_voltage, 4);
    memcpy(&packet[24], &telemetry.status_flags, 2);

    uint16_t crc = calculate_crc16(&packet[2], 24);
    packet[26] = crc & 0xFF;
    packet[27] = (crc >> 8) & 0xFF;

    _stream.write(packet, 28);
}
