#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
import rclpy.action
import math
import os
import sys

class SafetyWaypointNode(Node):
    def __init__(self):
        super().__init__('safety_waypoint_node')
        # 訂閱 /amcl_pose 以取得機器人目前位置
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.get_logger().info("已訂閱 /amcl_pose")
        
        # 修改訂閱 topic 為 /goal_pose 以更新原本目標
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 訂閱 nav_situation，根據此值來決定是否切換至安全點
        self.nav_situation_subscription = self.create_subscription(
            Int32,
            'nav_situation',
            self.nav_situation_callback,
            10
        )
        
        # 建立與 Nav2 NavigateToPose 的 action client 溝通
        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 安全點檔案路徑（請根據實際檔案位置設定）
        self.safety_points_file = "/home/laiting/HMR_WS/safety_point.txt"
        
        # 初始變數
        self.original_goal = None      # 由 /goal_pose 更新的原本目標
        self.nav_situation = None      # 當前 nav_situation 值
        self.last_amcl_pose = None     # 最新收到的 /amcl_pose
        
        # 狀態旗標
        self.safe_point_sent = False   # 是否已發送安全點目標

    def goal_callback(self, msg: PoseStamped):
        """
        接收到新的原本導航目標時更新 original_goal，
        並重設 safe_point_sent 狀態以便後續切換。
        """
        if self.original_goal is not None:
            old = self.original_goal.pose.position
            new = msg.pose.position
            # 當目標位置完全一致時不更新
            if old.x == new.x and old.y == new.y and old.z == new.z:
                return
        self.original_goal = msg
        self.safe_point_sent = False
        self.get_logger().info("更新原本目的地，新任務開始")

    def nav_situation_callback(self, msg: Int32):
        """
        當 nav_situation 為 2 時，啟動安全模式：
          - 若已有 /amcl_pose 且尚未發送安全點，則計算最近安全點並發送
        當 nav_situation 不為 2 時，
          - 若之前已發送安全點，則回復原本目標（原本目標由 /goal_pose 更新）。
        """
        self.nav_situation = msg.data
        self.get_logger().info(f"nav_situation 更新: {self.nav_situation}")
        if self.nav_situation == 2:
            # 安全模式：發送安全點
            if not self.safe_point_sent:
                if self.last_amcl_pose is not None:
                    safety_point = self.compute_closest_safety_point(self.last_amcl_pose)
                    if safety_point is not None:
                        self.send_goal(safety_point, stage="safety")
                        self.safe_point_sent = True
                        self.get_logger().info("已發送安全點目標 (safety mode)")
                    else:
                        self.get_logger().error("無法計算安全點")
                else:
                    self.get_logger().warn("尚未收到 /amcl_pose，無法計算安全點")
        else:
            # 當 nav_situation 不為 2 時，若之前已送出安全點，回復原本目標
            if self.safe_point_sent:
                if self.original_goal is not None:
                    orig_point = (
                        self.original_goal.pose.position.x,
                        self.original_goal.pose.position.y,
                        self.original_goal.pose.position.z
                    )
                    self.send_goal_no_wait(orig_point, stage="original")
                    self.get_logger().info("nav_situation 不為2，回復原本目標")
                else:
                    self.get_logger().warn("原本目標資料缺失，無法回復")
                self.safe_point_sent = False

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        當收到 /amcl_pose 時更新最新位置，
        若 nav_situation 處於安全模式且尚未發送安全點，則計算並發送安全點。
        """
        self.last_amcl_pose = msg
        if self.nav_situation == 2 and not self.safe_point_sent:
            safety_point = self.compute_closest_safety_point(msg)
            if safety_point is not None:
                self.send_goal(safety_point, stage="safety")
                self.safe_point_sent = True

    def compute_closest_safety_point(self, amcl_msg: PoseWithCovarianceStamped):
        """
        依照最新的 /amcl_pose 計算離機器人最近的安全點
        """
        current_x = amcl_msg.pose.pose.position.x
        current_y = amcl_msg.pose.pose.position.y
        current_z = amcl_msg.pose.pose.position.z
        self.get_logger().info(f"/amcl_pose 位置: ({current_x:.3f}, {current_y:.3f}, {current_z:.3f})")
        
        if not os.path.exists(self.safety_points_file):
            self.get_logger().error(f"檔案 {self.safety_points_file} 不存在")
            return None
        
        safety_points = []
        with open(self.safety_points_file, "r") as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue
                try:
                    parts = [p.strip() for p in line.split(',')]
                    x, y, z = map(float, parts)
                    safety_points.append((x, y, z))
                except Exception as e:
                    self.get_logger().error(f"解析安全點失敗: {line} ({str(e)})")
                    
        if not safety_points:
            self.get_logger().error("安全點檔案中未找到有效資料")
            return None
        
        # 選出最近的安全點
        min_dist = float("inf")
        closest_point = None
        for pt in safety_points:
            dist = math.sqrt((pt[0] - current_x) ** 2 +
                             (pt[1] - current_y) ** 2 +
                             (pt[2] - current_z) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_point = pt
        self.get_logger().info(f"選出的最近安全點: {closest_point} 距離: {min_dist:.3f}")
        return closest_point

    def send_goal(self, point, stage="safety"):
        """
        發送導航目標至 NavigateToPose 的 action server，
        包含回呼處理，用於安全或其他目的。
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = point[2]
        # 預設無旋轉
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"發送 {stage} 目標：{point}")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("等待 action server 超時")
            return
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, stage))
    
    def send_goal_no_wait(self, point, stage="original"):
        """
        非同步方式發送導航目標，不等待回呼。
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = point[2]
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"發送 {stage} 目標：{point}")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("等待 action server 超時")
            return
        self._action_client.send_goal_async(goal_msg)
    
    def goal_response_callback(self, future, stage):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{stage} 目標被拒絕")
            return
        self.get_logger().info(f"{stage} 目標已接受，等待執行結果...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self.get_result_callback(future, stage))
    
    def get_result_callback(self, future, stage):
        self.get_logger().info(f"{stage} 目標執行完畢")
        # 執行結果僅做紀錄，後續根據 nav_situation 進行切換，這裡不再自動關閉
    
    def feedback_callback(self, feedback_msg):
        # 若需處理導航回饋，可在此加入相關邏輯
        pass

    def shutdown_node(self):
        """
        如有需要可透過此方法關閉節點。
        """
        self.get_logger().info("關閉節點")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

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
