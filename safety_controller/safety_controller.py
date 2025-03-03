#!/usr/bin/env python3
from ast import Tuple
from math import pi
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
from safe_drive_msgs.msg import SafeDriveMsg


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.safety_subscriber = self.create_subscription(SafeDriveMsg, "/safety_topic", self.drive_msg_callback, 10)
        self.safety_publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)


    def drive_msg_callback(self, msg):
        """Processes the drive message."""

        line = msg.line.data
        drive_msg = msg.drive_msg.drive

        kp_gains = 0.5

        dist_to_wall = abs(line[1])/np.sqrt(line[0]**2 + 1)
        speed_multiplier = np.clip(1/(kp_gains*abs(line[0])), 0, 1)
        self.get_logger().info(f"Speed multiplier: {speed_multiplier}")
        new_speed = drive_msg.speed * speed_multiplier

        new_msg = AckermannDriveStamped()
        new_drive_msg = drive_msg
        new_drive_msg.speed = new_speed

        new_msg.drive = new_drive_msg
        new_msg.header = msg.drive_msg.header

        # Send drive
        self.safety_publisher.publish(new_msg)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
