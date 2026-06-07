#!/usr/bin/env python3
"""Relay Web UI flight commands to the UAV flight controller.

The Web UI must not publish PX4 control messages directly. It publishes only
high-level string commands on /web_ui/flight_command, and this UGV-side node
validates those commands before forwarding them to /flight_control/mission_cmd.
The UAV flight_control_node is the only node that turns these commands into
PX4 /fmu/in/* messages.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import String


COMMAND_ALIASES = {
    'ARM': 'ARM',
    'DISARM': 'DISARM',
    'TAKEOFF': 'TAKEOFF',
    'LAND': 'START_AUTO_LAND',
    'START_AUTO_LAND': 'START_AUTO_LAND',
    'EMERGENCY': 'EMERGENCY',
    'EMERGENCY_DISARM': 'EMERGENCY',
    'KILL': 'EMERGENCY',
}


class WebCommandRelay(Node):
    def __init__(self):
        super().__init__('web_command_relay')

        self.declare_parameter('web_command_topic', '/web_ui/flight_command')
        self.declare_parameter('mission_cmd_topic', '/flight_control/mission_cmd')
        self.declare_parameter('status_topic', '/web_ui/flight_command/status')

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.mission_pub = self.create_publisher(
            String,
            self.get_parameter('mission_cmd_topic').value,
            command_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            command_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('web_command_topic').value,
            self.command_cb,
            command_qos,
        )

        self.get_logger().info(
            'Web command relay started: '
            f"{self.get_parameter('web_command_topic').value} -> "
            f"{self.get_parameter('mission_cmd_topic').value}"
        )

    def command_cb(self, msg: String):
        raw_command = msg.data.strip()
        command_key = raw_command.upper()
        mission_command = COMMAND_ALIASES.get(command_key)
        if mission_command is None and command_key.startswith('ALIGN_FOR_LAND:'):
            mission_command = raw_command

        if mission_command is None:
            status = f'REJECTED:{raw_command}'
            self.status_pub.publish(String(data=status))
            self.get_logger().warn(
                f'Rejected unsupported Web UI command: {raw_command!r}')
            return

        self.mission_pub.publish(String(data=mission_command))
        self.status_pub.publish(
            String(data=f'ACCEPTED:{command_key}->{mission_command}'))
        self.get_logger().info(
            f'Forwarded Web UI command: {command_key} -> {mission_command}')


def main(args=None):
    rclpy.init(args=args)
    node = WebCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
