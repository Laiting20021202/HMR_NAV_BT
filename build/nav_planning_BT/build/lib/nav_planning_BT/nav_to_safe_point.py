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
        
        # 訂閱 my_pos_update 接收新的導航目標
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'my_pos_update',
            self.goal_callback,
            10
        )
        
        # 訂閱 BC_LV 主題 (假設型態為 std_msgs/Int32)
        self.bc_lv_subscription = self.create_subscription(
            Int32,
            'BC_LV',
            self.bc_lv_callback,
            10
        )
        
        # 建立與 Nav2 NavigateToPose 的 action client 溝通
        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 安全點檔案路徑
        self.safety_points_file = "/home/laiting/HMR_WS/safety_point.txt"
        
        # 初始變數
        self.original_goal = None      # 原本目標
        self.current_bc_lv = None       # 當前 BC_LV 數值
        
        # 狀態旗標
        self.safe_point_sent = False    # 是否已發送 safe point 目標
        self.mission_complete = True    # 任務是否已完成
        
        # timer 用以檢查 BC_LV 的值
        self._check_bc_lv_timer = None

    def goal_callback(self, msg: PoseStamped):
        """
        收到新的導航目標時，更新原本目的地並標記新任務開始。
        """
        if self.original_goal is not None:
            old = self.original_goal.pose.position
            new = msg.pose.position
            # 可依需求加入浮點數容差處理，這裡直接比較
            if old.x == new.x and old.y == new.y and old.z == new.z:
                return
        self.original_goal = msg
        # 標記新任務開始（未完成）
        self.mission_complete = False
        self.safe_point_sent = False
        self.get_logger().info("更新原本目的地，新任務開始")

    def bc_lv_callback(self, msg: Int32):
        """
        更新目前 BC_LV 數值。
        """
        self.current_bc_lv = msg.data
        self.get_logger().info(f"BC_LV 更新: {self.current_bc_lv}")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        收到機器人位置信息時：
         - 若已完成任務或 safe point 已發送則不重複動作。
         - 否則計算並發送安全點（safe point）導航目標。
        """
        if self.mission_complete or self.safe_point_sent:
            return
        if self.original_goal is None:
            self.get_logger().warn("尚未收到原本目的地，等待中...")
            return
        
        # 取得機器人目前位置
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z
        self.get_logger().info(f"目前位置: {current_x:.3f}, {current_y:.3f}, {current_z:.3f}")
        
        # 讀取並解析 safety_point.txt
        if not os.path.exists(self.safety_points_file):
            self.get_logger().error(f"檔案 {self.safety_points_file} 不存在")
            return
        
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
                    self.get_logger().error(f"解析 safety_point 失敗: {line} ({str(e)})")
                    
        if not safety_points:
            self.get_logger().error("安全點檔案中未找到有效資料")
            return
        
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
        
        # 發送 safe point 目標
        self.send_goal(closest_point, stage="safety")
        self.safe_point_sent = True

    def send_goal(self, point, stage="safety"):
        """
        傳統發送 goal 程式（包含回呼），用於發送安全點目標或其他目標。
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
        非等待方式發送原本目標，不設置回呼。
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
        if stage == "safety":
            # 安全點導航完成後，啟動 timer 檢查 BC_LV 是否達到 3
            self.get_logger().info("安全點導航完成，等待 BC_LV == 3...")
            self._check_bc_lv_timer = self.create_timer(0.5, self.check_bc_lv)
        else:
            self.get_logger().info("原本目標導航完畢")
            self.shutdown_node()
    
    def check_bc_lv(self):
        """
        檢查 BC_LV 是否達到 3，若是則發送原本目標並關閉節點。
        """
        if self.current_bc_lv == 3:
            self.get_logger().info("BC_LV 已等於 3，開始發送原本目標")
            if self._check_bc_lv_timer is not None:
                self._check_bc_lv_timer.cancel()
            if self.original_goal is None:
                self.get_logger().error("原本目標資料缺失")
                self.shutdown_node()
                return
            orig_point = (
                self.original_goal.pose.position.x,
                self.original_goal.pose.position.y,
                self.original_goal.pose.position.z
            )
            self.send_goal_no_wait(orig_point, stage="original")
            self.get_logger().info("原本目標已發送，關閉節點")
            self.shutdown_node()
        else:
            self.get_logger().info(f"等待中...目前 BC_LV 為: {self.current_bc_lv}")

    def feedback_callback(self, feedback_msg):
        # 如需處理導航回饋，可在此處加入邏輯
        pass

    def shutdown_node(self):
        """
        關閉節點：取消 timer、destroy_node()、shutdown 並強制結束程式。
        """
        self.get_logger().info("關閉節點")
        if self._check_bc_lv_timer is not None:
            self._check_bc_lv_timer.cancel()
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
