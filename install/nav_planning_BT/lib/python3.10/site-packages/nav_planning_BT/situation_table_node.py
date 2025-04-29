#!/usr/bin/env python3
# File: src/nav_planning_BT/nav_planning_BT/situation_table_node.py
# After editing, rebuild with:
#   colcon build --packages-select nav_planning_BT
#   source install/setup.bash

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
import rclpy.action
import math
import os

# Convert Euler angles to quaternion

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

# Extract yaw from quaternion

def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RoadsideParkingHandler:
    """
    Handles roadside parking (nav_situation == 1).
    Computes nearest wall point, applies margin, sends goal, restores original.
    """
    def __init__(self, node: Node, action_client, margin: float = 0.5):
        self.node = node
        self.client = action_client
        self.margin = margin
        self.last_amcl = None
        self.last_scan = None
        self.sent = False

        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        node.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        node.create_subscription(Int32, 'nav_situation', self.nav_cb, 10)

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.last_amcl = msg

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def nav_cb(self, msg: Int32):
        mode = msg.data
        if mode == 1:
            if not self.sent and self.last_amcl and self.last_scan:
                pt = self.compute_margin_wall_point(self.last_amcl, self.last_scan)
                if pt:
                    self.node.send_goal(pt, stage='roadside')
                    self.sent = True
                    self.node.get_logger().info(f'Roadside goal sent with margin {self.margin}')
            return
        if self.sent:
            orig = self.node.original_goal
            if orig:
                x = orig.pose.position.x
                y = orig.pose.position.y
                z = orig.pose.position.z
                self.node.send_goal_no_wait((x, y, z), stage='original')
                self.node.get_logger().info('Restored original goal')
            self.sent = False

    def compute_margin_wall_point(self, amcl_msg: PoseWithCovarianceStamped, scan_msg: LaserScan):
        robot = amcl_msg.pose.pose
        min_r, idx = float('inf'), -1
        for i, r in enumerate(scan_msg.ranges):
            if scan_msg.range_min < r < scan_msg.range_max and r < min_r:
                min_r, idx = r, i
        if idx < 0:
            self.node.get_logger().warn('No valid laser data')
            return None

        r_adj = max(min_r - self.margin, 0.0)
        angle = scan_msg.angle_min + idx * scan_msg.angle_increment
        x_local = r_adj * math.cos(angle)
        y_local = r_adj * math.sin(angle)
        yaw = quaternion_to_yaw(robot.orientation)
        gx = robot.position.x + (x_local * math.cos(yaw) - y_local * math.sin(yaw))
        gy = robot.position.y + (x_local * math.sin(yaw) + y_local * math.cos(yaw))
        return (gx, gy, 0.0, yaw)


class SafetyWaypointNode(Node):
    """
    Handles nav_situation == 2: selects closest safety point and uses RoadsideParkingHandler.
    """
    def __init__(self):
        super().__init__('safety_waypoint_node')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Int32, 'nav_situation', self.nav_situation_callback, 10)

        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.safety_points_file = os.path.expanduser('~/HMR_WS/safety_point.txt')
        self.original_goal = None
        self.nav_situation = None
        self.last_amcl_pose = None
        self.safe_point_sent = False
        self.roadside = RoadsideParkingHandler(self, self._action_client, margin=0.5)
        self.get_logger().info('SafetyWaypointNode initialized')

    def goal_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        if self.original_goal:
            old = self.original_goal.pose.position
            if old.x == pos.x and old.y == pos.y and old.z == pos.z:
                return
        self.original_goal = msg
        self.safe_point_sent = False
        self.roadside.sent = False
        self.get_logger().info('Original goal updated')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg
        if self.nav_situation == 2 and not self.safe_point_sent:
            pt = self.compute_closest_safety_point(msg)
            if pt:
                self.send_goal(pt, stage='safety')
                self.safe_point_sent = True
                self.get_logger().info('Safety goal sent')

    def nav_situation_callback(self, msg: Int32):
        self.nav_situation = msg.data
        self.get_logger().info(f'nav_situation updated: {self.nav_situation}')
        self.roadside.nav_cb(msg)
        if self.nav_situation == 2:
            if self.last_amcl_pose and not self.safe_point_sent:
                pt = self.compute_closest_safety_point(self.last_amcl_pose)
                if pt:
                    self.send_goal(pt, stage='safety')
                    self.safe_point_sent = True
                    self.get_logger().info('Safety goal sent')
        else:
            if self.safe_point_sent:
                if self.original_goal:
                    coords = self.original_goal.pose.position
                    self.send_goal_no_wait((coords.x, coords.y, coords.z), stage='original')
                    self.get_logger().info('Restored original goal')
                self.safe_point_sent = False

    def compute_closest_safety_point(self, amcl_msg: PoseWithCovarianceStamped):
        pos = amcl_msg.pose.pose.position
        if not os.path.isfile(self.safety_points_file):
            self.get_logger().error('Safety points file not found')
            return None
        points, yaws = [], []
        with open(self.safety_points_file, 'r') as f:
            for line in f:
                parts = [p.strip() for p in line.split(',')]
                if len(parts) != 4:
                    continue
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    yaw = float(parts[3])
                except ValueError:
                    continue
                points.append((x, y, z))
                yaws.append(yaw)
        if not points:
            self.get_logger().error('No valid safety points')
            return None
        dists = [math.hypot(x-pos.x, y-pos.y) for x, y, z in points]
        idx = dists.index(min(dists))
        bx, by, bz = points[idx]
        byaw = yaws[idx]
        return (bx, by, bz, byaw)

    def send_goal(self, point, stage='safety'):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        x, y, z, yaw = point
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server unavailable')
            return
        self._action_client.send_goal_async(goal)

    def send_goal_no_wait(self, point, stage='original'):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z = point
        goal.pose.pose.orientation.w = 1.0
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server unavailable')
            return
        self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
