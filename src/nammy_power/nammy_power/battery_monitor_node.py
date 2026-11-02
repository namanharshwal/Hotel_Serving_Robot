#!/usr/bin/env python3
"""
24V 15Ah LiFePO4 Battery Monitor Node for Nammy Robot Platform.
Computes State-of-Charge (SOC), per-cell voltages, and battery health alerts.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        self.declare_parameter('nominal_voltage', 25.6)
        self.declare_parameter('cells_series', 8)           # 8S LFP
        self.declare_parameter('capacity_ah', 15.0)         # 15 Ah (384 Wh)
        self.declare_parameter('low_voltage_threshold', 21.6)
        self.declare_parameter('critical_voltage_threshold', 20.0)

        self.v_nominal = self.get_parameter('nominal_voltage').value
        self.num_cells = self.get_parameter('cells_series').value
        self.v_low = self.get_parameter('low_voltage_threshold').value
        self.v_critical = self.get_parameter('critical_voltage_threshold').value

        self.current_voltage = 26.4
        self.current_pct = 0.85

        self.batt_sub = self.create_subscription(
            BatteryState, 'nammy/battery_state', self.battery_callback, 10
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.alert_pub = self.create_publisher(String, 'nammy/power/alert', 10)

        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def battery_callback(self, msg: BatteryState):
        self.current_voltage = msg.voltage
        self.current_pct = msg.percentage

        # Trigger critical cutoff alert if voltage drops below safe threshold
        if self.current_voltage < self.v_critical:
            alert = String()
            alert.data = f"CRITICAL: Battery voltage {self.current_voltage:.2f}V < {self.v_critical}V! Cutoff imminent!"
            self.alert_pub.publish(alert)
            self.get_logger().error(alert.data)
        elif self.current_voltage < self.v_low:
            alert = String()
            alert.data = f"WARNING: Low battery ({self.current_voltage:.2f}V / {int(self.current_pct*100)}%). Recharge required."
            self.alert_pub.publish(alert)

    def publish_diagnostics(self):
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = self.get_clock().now().to_msg()

        stat = DiagnosticStatus()
        stat.name = "Power Subsystem: 24V LiFePO4"
        stat.hardware_id = "LFP_8S_32140_15Ah"

        if self.current_voltage < self.v_critical:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Critical Under-voltage! System Protection Triggered."
        elif self.current_voltage < self.v_low:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Battery Low. Return to charge dock."
        else:
            stat.level = DiagnosticStatus.OK
            stat.message = "Battery Nominal Operating Range."

        v_cell = self.current_voltage / self.num_cells
        stat.values = [
            KeyValue(key="Pack Voltage", value=f"{self.current_voltage:.2f} V"),
            KeyValue(key="Cell Average", value=f"{v_cell:.3f} V"),
            KeyValue(key="State of Charge", value=f"{int(self.current_pct * 100)} %"),
            KeyValue(key="Chemistry", value="LiFePO4 (8S)"),
            KeyValue(key="Rated Capacity", value="15 Ah (384 Wh)"),
            KeyValue(key="BMS Limit", value="20A Continuous")
        ]

        diag_arr.status.append(stat)
        self.diag_pub.publish(diag_arr)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
