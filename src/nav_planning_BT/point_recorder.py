#!/usr/bin/env python3
import os
import numpy as np
# Monkey-patch deprecated numpy alias to support transforms3d
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

class GoalPoseRecorder(Node):
    def __init__(self):
        super().__init__('goal_pose_recorder')
        # 訂閱 /goal_pose 主題（型態為 PoseStamped）
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # 絕對 topic 名稱，避免命名空間問題
            self.goal_pose_callback,
            10  # 預設 QoS 佇列大小
        )
        self.get_logger().info("已訂閱 /goal_pose")

        # 設定檔案路徑，並確保所在資料夾存在
        self.file_path = "/home/laiting/HMR_WS/safety_point.txt"
        dir_path = os.path.dirname(self.file_path)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

    def goal_pose_callback(self, msg: PoseStamped):
        """
        紀錄位置與朝向資訊：x, y, z, yaw
        格式：4.118, 0.487, 0.000, 1.571
        """
        # 取得位置
        pos = msg.pose.position
        # 取得 quaternion 並轉換為 Euler 角，只取 yaw
        ori = msg.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(quat)

        # 組成一行文字
        line = "{:.3f}, {:.3f}, {:.3f}, {:.3f}\n".format(
            pos.x, pos.y, pos.z,
            yaw
        )

        # 寫入檔案
        try:
            with open(self.file_path, "a") as f:
                f.write(line)
            self.get_logger().info("記錄 goal_pose 位置與朝向 (yaw): " + line.strip())
        except Exception as e:
            self.get_logger().error("寫入檔案時發生錯誤: " + str(e))


def main():
    rclpy.init(args=None)
    node = GoalPoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("結束 goal_pose 記錄")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
