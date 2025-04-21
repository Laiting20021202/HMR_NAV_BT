#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
import rclpy.action
import math
import os
import sys

def quaternion_to_yaw(q):
    """Convert a quaternion into a yaw angle."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class RoadsideParkingHandler:
    """
    Encapsulates the roadside‐parking behavior (nav_situation == 1).
    Computes the nearest wall point from /scan + /amcl_pose, applies a margin,
    and issues a NavigateToPose goal. Restores the original goal when exiting.
    """
    def __init__(self, node: Node, action_client, margin: float = 0.5):
        self.node = node
        self.client = action_client
        self.margin = margin

        self.last_amcl = None
        self.last_scan = None
        self.sent = False

        # subscriptions
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )
        self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )
        self.node.create_subscription(
            Int32,
            'nav_situation',
            self.nav_cb,
            10
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.last_amcl = msg

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def nav_cb(self, msg: Int32):
        nav = msg.data
        # enter roadside‐parking mode
        if nav == 1:
            if not self.sent and self.last_amcl and self.last_scan:
                pt = self.compute_margin_wall_point(self.last_amcl, self.last_scan)
                if pt:
                    self.node.send_goal(pt, stage="roadside")
                    self.sent = True
                    self.node.get_logger().info(
                        f"[roadside] goal sent with margin {self.margin} m"
                    )
            return

        # exit roadside‐parking: restore original goal
        if self.sent:
            orig = self.node.original_goal
            if orig:
                x = orig.pose.position.x
                y = orig.pose.position.y
                z = orig.pose.position.z
                self.node.send_goal_no_wait((x, y, z), stage="original")
                self.node.get_logger().info("[roadside] exiting, restored original goal")
            else:
                self.node.get_logger().warn("[roadside] no original goal to restore")
            self.sent = False

    def compute_margin_wall_point(self,
                                  amcl_msg: PoseWithCovarianceStamped,
                                  scan_msg: LaserScan):
        """
        Find the closest valid scan point, subtract margin from the range,
        and transform into map coords.
        """
        robot = amcl_msg.pose.pose

        # find min valid range
        min_r, idx = float('inf'), -1
        for i, r in enumerate(scan_msg.ranges):
            if scan_msg.range_min < r < scan_msg.range_max and r < min_r:
                min_r, idx = r, i
        if idx < 0:
            self.node.get_logger().warn("[roadside] no valid laser data")
            return None

        # apply margin
        r_adj = max(min_r - self.margin, 0.0)
        angle = scan_msg.angle_min + idx * scan_msg.angle_increment
        x_local = r_adj * math.cos(angle)
        y_local = r_adj * math.sin(angle)

        yaw = quaternion_to_yaw(robot.orientation)
        gx = robot.position.x + (x_local * math.cos(yaw)
                                 - y_local * math.sin(yaw))
        gy = robot.position.y + (x_local * math.sin(yaw)
                                 + y_local * math.cos(yaw))

        self.node.get_logger().info(
            f"[roadside] wall point at ({gx:.2f}, {gy:.2f}, 0.0)"
        )
        return (gx, gy, 0.0)

class SafetyWaypointNode(Node):
    """Original node for nav_situation == 2 (safety‐point) plus integration."""
    def __init__(self):
        super().__init__('safety_waypoint_node')

        # subscribe original topics
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.get_logger().info("已訂閱 /amcl_pose")

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.create_subscription(
            Int32,
            'nav_situation',
            self.nav_situation_callback,
            10
        )

        # Nav2 action client
        self._action_client = rclpy.action.ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # safety‐point file
        self.safety_points_file = "/home/laiting/HMR_WS/safety_point.txt"

        # state
        self.original_goal  = None
        self.nav_situation  = None
        self.last_amcl_pose = None
        # flags
        self.safe_point_sent = False

        # instantiate roadside handler with desired margin
        self.roadside = RoadsideParkingHandler(self, self._action_client, margin=0.5)

    def goal_callback(self, msg: PoseStamped):
        if self.original_goal is not None:
            old = self.original_goal.pose.position
            new = msg.pose.position
            if old.x == new.x and old.y == new.y and old.z == new.z:
                return
        self.original_goal = msg
        self.safe_point_sent = False
        # also reset roadside in case
        self.roadside.sent = False
        self.get_logger().info("更新原本目的地，新任務開始")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg
        # handle safety‐point mode
        if self.nav_situation == 2 and not self.safe_point_sent:
            pt = self.compute_closest_safety_point(msg)
            if pt:
                self.send_goal(pt, stage="safety")
                self.safe_point_sent = True
                self.get_logger().info("已發送安全點目標 (safety mode)")

    def nav_situation_callback(self, msg: Int32):
        self.nav_situation = msg.data
        self.get_logger().info(f"nav_situation 更新: {self.nav_situation}")
        # safety‐point logic only; roadside parking is fully handled by its handler
        if self.nav_situation == 2:
            if not self.safe_point_sent and self.last_amcl_pose:
                pt = self.compute_closest_safety_point(self.last_amcl_pose)
                if pt:
                    self.send_goal(pt, stage="safety")
                    self.safe_point_sent = True
                    self.get_logger().info("已發送安全點目標 (safety mode)")
        else:
            if self.safe_point_sent:
                if self.original_goal:
                    x = self.original_goal.pose.position.x
                    y = self.original_goal.pose.position.y
                    z = self.original_goal.pose.position.z
                    self.send_goal_no_wait((x, y, z), stage="original")
                    self.get_logger().info("nav_situation 不為2，回復原本目標")
                else:
                    self.get_logger().warn("原本目標資料缺失，無法回復")
                self.safe_point_sent = False

    def compute_closest_safety_point(self,
                                     amcl_msg: PoseWithCovarianceStamped):
        pos = amcl_msg.pose.pose.position
        self.get_logger().info(
            f"/amcl_pose 位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
        )
        if not os.path.exists(self.safety_points_file):
            self.get_logger().error(f"檔案 {self.safety_points_file} 不存在")
            return None

        points = []
        with open(self.safety_points_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    x, y, z = map(float, line.split(','))
                    points.append((x, y, z))
                except Exception as e:
                    self.get_logger().error(
                        f"解析安全點失敗: {line} ({e})"
                    )

        if not points:
            self.get_logger().error("安全點檔案中未找到有效資料")
            return None

        best = min(points, key=lambda pt: math.hypot(pt[0]-pos.x, pt[1]-pos.y))
        dist = math.hypot(best[0]-pos.x, best[1]-pos.y)
        self.get_logger().info(f"選出的最近安全點: {best} 距離: {dist:.3f}")
        return best

    def send_goal(self, point, stage="safety"):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        goal.pose.pose.position.z = point[2]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"發送 {stage} 目標：{point}")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("等待 action server 超時")
            return
        fut = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        fut.add_done_callback(lambda f: self.goal_response_callback(f, stage))

    def send_goal_no_wait(self, point, stage="original"):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        goal.pose.pose.position.z = point[2]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"發送 {stage} 目標：{point}")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("等待 action server 超時")
            return
        self._action_client.send_goal_async(goal)

    def goal_response_callback(self, future, stage):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f"{stage} 目標被拒絕")
            return
        self.get_logger().info(f"{stage} 目標已接受，等待執行結果…")
        fut = handle.get_result_async()
        fut.add_done_callback(
            lambda f: self.get_result_callback(f, stage)
        )

    def get_result_callback(self, future, stage):
        self.get_logger().info(f"{stage} 目標執行完畢")

    def feedback_callback(self, _):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SafetyWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
