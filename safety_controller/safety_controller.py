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

        self.declare_parameter("side", -1)
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.safety_subscriber = self.create_subscription(AckermannDriveStamped, "/vesc/low_level/ackermann_cmd", self.drive_msg_callback, 10)
        self.safety_publisher = self.create_publisher(AckermannDriveStamped, "/vesc/low_level/input/safety", 10)
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value

        self.scan_length = None
        self.max_speed = 1.5
        self.speed = 1.5

    def laser_scan_callback(self, msg):
        # self.get_logger().info("Scanning")
        if not self.scan_length:
            self.scan_length = len(msg.ranges)
            modifier = round(self.scan_length*(135/260))
            self.r1 = round(self.scan_length*(1 - (45/260)))
            self.r0 = self.r1 - modifier
            self.l0 = round(self.scan_length*(45/260))
            self.l1 = self.l0 + modifier

        scan = msg
        ranges = scan.ranges

        # hardcoded range values for sim testing
        # right_range = (31, 83)
        # left_range = (17, 69)
    
        # calculated range values
        right_range = (self.r0, self.r1)
        left_range = (self.l0, self.l1)

        HARD_STOP_BOUND = 0.4
        SLOW_BOUND = 0.8

        if self.SIDE == -1:
            rng = range(*right_range, 2)
        else:
            rng = range(*left_range, 2)

        self.speed = self.max_speed

        for k in rng:
            average = 1/2 * (ranges[k] + ranges[k+1])
            if average < HARD_STOP_BOUND:
                self.speed = 0.0
                break
            if average < SLOW_BOUND:
                self.speed = .5 * self.max_speed
        



    def drive_msg_callback(self, msg):
        """Processes the drive message."""
        # get range slicing boundaries

        new_msg = AckermannDriveStamped()
        new_msg.header = msg.header
        new_msg.drive = msg.drive
        new_msg.drive.speed = self.speed

        # Send drive
        self.get_logger().info("self.speed: " + str(self.speed))
        self.safety_publisher.publish(new_msg)
        # self.get_logger().info("Published drive message")




def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
