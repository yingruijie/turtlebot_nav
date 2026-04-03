import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class TurtlebotNavNode(Node):
    def __init__(self):
        super().__init__('turtlebot_nav')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.waypoints = np.asarray([
            [1.25, -1.26],
            [1.92, 0.04],
            [2.92, -1.42],
            [2.53, 2.14],
            [0.45, 2.20],
            [1.28, 1.25],
        ])
        self.current_waypoint_idx = 0
        self.pose_ready = False
        self.scan_ready = False
        self.goal_reached = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan_msg = None
        self.turn_preference = -1.0

        self.goal_tolerance = 0.15
        self.max_linear_speed = 0.16
        self.max_angular_speed = 1.0
        self.turn_only_threshold = 0.9
        self.min_repulsion_distance = 0.12
        self.repulsion_radius = 0.60
        self.front_slow_radius = 0.45
        self.attraction_gain = 1.0
        self.repulsion_gain = 0.06
        self.max_repulsion_norm = 1.2
        self.minimum_turn_speed = 0.2
        self.backward_turn_deadband = 0.18

        self.debug_counter = 0
        self.control_timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('Turtlebot potential-field navigation node started.')

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        self.yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose_ready = True

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg
        self.scan_ready = True

    def navigate(self):
        self.debug_counter += 1

        if self.goal_reached:
            self.publish_cmd(0.0, 0.0)
            return

        if not self.pose_ready or not self.scan_ready:
            if self.debug_counter % 20 == 0:
                self.get_logger().info(
                    f'Waiting for sensors: pose_ready={self.pose_ready}, scan_ready={self.scan_ready}'
                )
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.goal_reached = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('All waypoints reached. Stopping robot.')
            return

        target = self.waypoints[self.current_waypoint_idx]
        goal_vector_world = np.asarray([target[0] - self.x, target[1] - self.y], dtype=float)
        distance = float(np.linalg.norm(goal_vector_world))

        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx}: [{target[0]:.2f}, {target[1]:.2f}]'
            )
            self.current_waypoint_idx += 1
            self.publish_cmd(0.0, 0.0)
            return

        goal_vector_body = self.rotate_world_to_body(goal_vector_world)
        attraction = self.attraction_gain * goal_vector_body / max(distance, 1e-6)
        repulsion = self.compute_repulsion_vector(self.scan_msg)
        total_vector = attraction + repulsion

        heading_error = self.compute_heading_error(total_vector, goal_vector_body)
        front_distance = self.get_front_distance(self.scan_msg)

        linear_speed = min(self.max_linear_speed, 0.35 * distance)
        linear_speed *= max(0.0, math.cos(heading_error))
        if abs(heading_error) > self.turn_only_threshold:
            linear_speed = 0.0
        if front_distance < self.front_slow_radius:
            slow_ratio = (front_distance - self.min_repulsion_distance) / max(
                self.front_slow_radius - self.min_repulsion_distance,
                1e-6,
            )
            linear_speed *= float(np.clip(slow_ratio, 0.0, 1.0))

        angular_speed = float(np.clip(1.8 * heading_error, -self.max_angular_speed, self.max_angular_speed))
        if abs(heading_error) > 0.12 and abs(angular_speed) < self.minimum_turn_speed:
            angular_speed = math.copysign(self.minimum_turn_speed, heading_error)

        state = 'tracking'
        if linear_speed < 1e-3 and abs(angular_speed) > 0.05:
            state = 'turning'
        elif np.linalg.norm(repulsion) > 0.05:
            state = 'avoiding'

        self.maybe_log_status(
            state,
            target,
            distance,
            heading_error,
            linear_speed,
            angular_speed,
            attraction,
            repulsion,
            total_vector,
            front_distance,
        )
        self.publish_cmd(linear_speed, angular_speed)

    def compute_repulsion_vector(self, scan: LaserScan) -> np.ndarray:
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid_mask = (
            np.isfinite(ranges)
            & (ranges > max(scan.range_min, self.min_repulsion_distance))
            & (ranges < min(scan.range_max, self.repulsion_radius))
        )
        if not np.any(valid_mask):
            return np.zeros(2, dtype=float)

        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)
        distances = np.hypot(x_points, y_points)

        # Only obstacles in the front half-plane should steer the robot.
        front_mask = x_points > 0.0
        if not np.any(front_mask):
            return np.zeros(2, dtype=float)

        x_points = x_points[front_mask]
        y_points = y_points[front_mask]
        distances = distances[front_mask]

        forward_weight = np.clip(x_points / distances, 0.0, 1.0) ** 2
        weights = (
            self.repulsion_gain
            * forward_weight
            * (1.0 / distances - 1.0 / self.repulsion_radius)
            / (distances ** 2)
        )

        obstacle_vectors = np.stack((x_points, y_points), axis=1)
        repulsion = -np.sum(weights[:, None] * obstacle_vectors / distances[:, None], axis=0)

        norm = float(np.linalg.norm(repulsion))
        if norm > self.max_repulsion_norm:
            repulsion *= self.max_repulsion_norm / norm
        return repulsion

    def compute_heading_error(self, total_vector: np.ndarray, goal_vector_body: np.ndarray) -> float:
        lateral = float(total_vector[1])
        longitudinal = float(total_vector[0])

        if abs(lateral) > 0.05:
            self.turn_preference = math.copysign(1.0, lateral)
        elif abs(goal_vector_body[1]) > 0.05:
            self.turn_preference = math.copysign(1.0, goal_vector_body[1])

        if longitudinal < 0.0 and abs(lateral) < self.backward_turn_deadband:
            return self.turn_preference * (math.pi - 0.15)

        return self.normalize_angle(math.atan2(lateral, longitudinal))

    def get_front_distance(self, scan: LaserScan) -> float:
        front_a = self.get_sector_min_distance(scan, 345.0, 360.0)
        front_b = self.get_sector_min_distance(scan, 0.0, 15.0)
        return min(front_a, front_b)

    def maybe_log_status(
        self,
        state: str,
        target,
        distance: float,
        heading_error: float,
        linear_speed: float,
        angular_speed: float,
        attraction: np.ndarray,
        repulsion: np.ndarray,
        total_vector: np.ndarray,
        front_distance: float,
    ):
        if self.debug_counter % 10 != 0:
            return

        self.get_logger().info(
            'state=%s wp=%d pos=(%.2f, %.2f) target=(%.2f, %.2f) dist=%.2f '
            'yaw=%.2f head_err=%.2f cmd=(%.2f, %.2f) front=%.2f pref=%.0f '
            'att=(%.2f, %.2f) rep=(%.2f, %.2f) total=(%.2f, %.2f)'
            % (
                state,
                self.current_waypoint_idx,
                self.x,
                self.y,
                target[0],
                target[1],
                distance,
                self.yaw,
                heading_error,
                linear_speed,
                angular_speed,
                front_distance,
                self.turn_preference,
                attraction[0],
                attraction[1],
                repulsion[0],
                repulsion[1],
                total_vector[0],
                total_vector[1],
            )
        )

    def rotate_world_to_body(self, vector_world: np.ndarray) -> np.ndarray:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return np.asarray([
            cos_yaw * vector_world[0] + sin_yaw * vector_world[1],
            -sin_yaw * vector_world[0] + cos_yaw * vector_world[1],
        ])

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_sector_min_distance(self, scan: LaserScan, start_deg: float, end_deg: float) -> float:
        if not scan.ranges:
            return math.inf

        angle_min_deg = math.degrees(scan.angle_min)
        angle_increment_deg = math.degrees(scan.angle_increment)
        if angle_increment_deg == 0.0:
            return math.inf

        start_deg = start_deg % 360.0
        end_deg = end_deg % 360.0
        sample_angles = angle_min_deg + np.arange(len(scan.ranges)) * angle_increment_deg
        sample_angles = np.mod(sample_angles, 360.0)

        if start_deg <= end_deg:
            mask = (sample_angles >= start_deg) & (sample_angles <= end_deg)
        else:
            mask = (sample_angles >= start_deg) | (sample_angles <= end_deg)

        sector_ranges = np.asarray(scan.ranges)[mask]
        valid_ranges = sector_ranges[
            np.isfinite(sector_ranges)
            & (sector_ranges > scan.range_min)
            & (sector_ranges < scan.range_max)
        ]
        if valid_ranges.size == 0:
            return math.inf

        return float(np.min(valid_ranges))


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    turtlebot_nav_node = TurtlebotNavNode()
    try:
        rclpy.spin(turtlebot_nav_node)
    except KeyboardInterrupt:
        pass
    turtlebot_nav_node.publish_cmd(0.0, 0.0)
    turtlebot_nav_node.destroy_node()
    rclpy.shutdown()
