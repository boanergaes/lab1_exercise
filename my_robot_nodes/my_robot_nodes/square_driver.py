#!/usr/bin/env python3
"""Action client that drives the robot around a square.

Calls /drive_distance to drive forward for side_length meters, then /rotate to turn
a quarter turn to the left, four times over.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def drive_forward(self, distance):
        self.get_logger().info("Waiting for /drive_distance action server...")
        self.drive_client.wait_for_server()

        goal = DriveDistance.Goal()
        goal.distance = float(distance)

        for attempt in range(5):
            send_future = self.drive_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if goal_handle.accepted:
                break
            self.get_logger().warn(
                f"Drive goal rejected (attempt {attempt + 1}/5), retrying..."
            )
            time.sleep(1.0)
        else:
            self.get_logger().error("Drive goal rejected after 5 attempts")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def turn_left(self, angle=math.pi / 2):
        self.get_logger().info("Waiting for /rotate action server...")
        self.rotate_client.wait_for_server()

        goal = Rotate.Goal()
        goal.angle = float(angle)

        for attempt in range(5):
            send_future = self.rotate_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if goal_handle.accepted:
                break
            self.get_logger().warn(
                f"Rotate goal rejected (attempt {attempt + 1}/5), retrying..."
            )
            time.sleep(1.0)
        else:
            self.get_logger().error("Rotate goal rejected after 5 attempts")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def drive_square(self):
        side_length = self.get_parameter("side_length").value
        self.get_logger().info(
            f"Starting square driver with side_length = {side_length:.2f} m"
        )

        for side in range(1, 5):
            self.get_logger().info(f"--- Side {side}/4: Driving forward {side_length:.2f} m ---")
            if not self.drive_forward(side_length):
                self.get_logger().error(f"Driving failed on side {side}")
                return False

            self.get_logger().info(f"--- Corner {side}/4: Rotating left 90 degrees ---")
            if not self.turn_left(math.pi / 2):
                self.get_logger().error(f"Rotation failed after side {side}")
                return False

        self.get_logger().info("Square drive completed successfully!")
        return True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.drive_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
