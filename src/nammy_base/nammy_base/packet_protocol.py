#!/usr/bin/env python3
"""
Framed Binary Serial Protocol for Nammy Robot (Jetson Orin <-> ESP32-S3)
Provides high-speed, noise-immune packet framing with CRC16 integrity checks.
"""

import struct
from typing import Optional, Tuple, NamedTuple

HEADER_BYTE_1 = 0xAA
HEADER_BYTE_2 = 0x55

# Message Types
MSG_CMD_VEL   = 0x01
MSG_TELEMETRY = 0x02
MSG_HEARTBEAT = 0x03
MSG_ESTOP     = 0x04
MSG_PID_TUNE  = 0x05
MSG_RAW_PWM   = 0x06

class TelemetryData(NamedTuple):
    left_pos_rad: float
    right_pos_rad: float
    left_vel_rad_s: float
    right_vel_rad_s: float
    battery_volts: float
    status_flags: int
    sonar_front_m: float
    sonar_left_m: float
    sonar_right_m: float
    sonar_rear_m: float

def crc16_ccitt(data: bytes) -> int:
    """Calculate 16-bit CRC-CCITT (polynomial 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_cmd_vel(left_rad_s: float, right_rad_s: float) -> bytes:
    """Pack velocity command into framed packet."""
    payload = struct.pack('<ff', left_rad_s, right_rad_s)
    length = len(payload)
    header_and_payload = struct.pack('<BBBB', HEADER_BYTE_1, HEADER_BYTE_2, length, MSG_CMD_VEL) + payload
    crc = crc16_ccitt(header_and_payload[2:])
    return header_and_payload + struct.pack('<H', crc)

def pack_estop() -> bytes:
    """Pack immediate emergency stop command."""
    length = 0
    header_and_payload = struct.pack('<BBBB', HEADER_BYTE_1, HEADER_BYTE_2, length, MSG_ESTOP)
    crc = crc16_ccitt(header_and_payload[2:])
    return header_and_payload + struct.pack('<H', crc)

def pack_heartbeat() -> bytes:
    """Pack keepalive heartbeat."""
    length = 0
    header_and_payload = struct.pack('<BBBB', HEADER_BYTE_1, HEADER_BYTE_2, length, MSG_HEARTBEAT)
    crc = crc16_ccitt(header_and_payload[2:])
    return header_and_payload + struct.pack('<H', crc)

def pack_pid_tune(kp: float, ki: float, kd: float, kff: float) -> bytes:
    """Pack live PID gain tuning command."""
    payload = struct.pack('<ffff', kp, ki, kd, kff)
    length = len(payload)
    header_and_payload = struct.pack('<BBBB', HEADER_BYTE_1, HEADER_BYTE_2, length, MSG_PID_TUNE) + payload
    crc = crc16_ccitt(header_and_payload[2:])
    return header_and_payload + struct.pack('<H', crc)

def pack_raw_pwm(left_duty: int, right_duty: int) -> bytes:
    """Pack direct open-loop PWM duty cycle (-1023 to 1023)."""
    payload = struct.pack('<hh', left_duty, right_duty)
    length = len(payload)
    header_and_payload = struct.pack('<BBBB', HEADER_BYTE_1, HEADER_BYTE_2, length, MSG_RAW_PWM) + payload
    crc = crc16_ccitt(header_and_payload[2:])
    return header_and_payload + struct.pack('<H', crc)

class PacketParser:
    """Stateful stream parser for incoming ESP32-S3 packets."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk: bytes):
        self.buffer.extend(chunk)

    def parse(self) -> Optional[Tuple[int, bytes]]:
        """Extract next valid packet from internal buffer."""
        while len(self.buffer) >= 6:  # Minimum packet size (2 hdr + 1 len + 1 type + 2 crc)
            # Find start header
            if self.buffer[0] != HEADER_BYTE_1 or self.buffer[1] != HEADER_BYTE_2:
                self.buffer.pop(0)
                continue

            payload_len = self.buffer[2]
            total_len = 2 + 1 + 1 + payload_len + 2  # H1, H2, LEN, TYPE, PAYLOAD..., CRC_L, CRC_H

            if len(self.buffer) < total_len:
                return None  # Wait for more bytes

            packet_bytes = self.buffer[:total_len]
            msg_type = packet_bytes[3]
            payload = packet_bytes[4:4 + payload_len]
            received_crc = struct.unpack('<H', packet_bytes[-2:])[0]

            # Verify CRC
            calculated_crc = crc16_ccitt(packet_bytes[2:-2])
            if calculated_crc == received_crc:
                # Valid packet
                del self.buffer[:total_len]
                return (msg_type, bytes(payload))
            else:
                # Corrupt packet, advance by 1 byte to resynchronize
                self.buffer.pop(0)

        return None

def unpack_telemetry(payload: bytes) -> Optional[TelemetryData]:
    """Unpack telemetry payload (30 bytes: 5 floats + 1 uint16 + 4 uint16 sonars in mm)."""
    if len(payload) == 30:
        left_pos, right_pos, left_vel, right_vel, batt_volts, flags, sf_mm, sl_mm, sr_mm, srear_mm = struct.unpack('<fffffHHHHH', payload)
        return TelemetryData(
            left_pos, right_pos, left_vel, right_vel, batt_volts, flags,
            sf_mm / 1000.0, sl_mm / 1000.0, sr_mm / 1000.0, srear_mm / 1000.0
        )
    elif len(payload) == 22:
        left_pos, right_pos, left_vel, right_vel, batt_volts, flags = struct.unpack('<fffffH', payload)
        return TelemetryData(left_pos, right_pos, left_vel, right_vel, batt_volts, flags, 0.0, 0.0, 0.0, 0.0)
    return None
