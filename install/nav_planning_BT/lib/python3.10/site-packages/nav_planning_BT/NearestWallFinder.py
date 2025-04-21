#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

# 將 quaternion 轉為 yaw 角
def quaternion_to_yaw(q):
    # 使用公式：yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class NearestWallFinder(Node):
    def __init__(self):
        super().__init__('nearest_wall_finder_manual')
        
        # 訂閱 /amcl_pose 用來獲取機器人於世界（map）的位姿
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        # 訂閱 /scan 取得雷射掃描
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.robot_pose = None  # 最新的機器人 pose (map 座標)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        # 從 /amcl_pose 中取得機器人位置與方向
        self.robot_pose = msg.pose.pose
        self.get_logger().debug(
            f"機器人位置更新: x = {self.robot_pose.position.x:.2f}, y = {self.robot_pose.position.y:.2f}"
        )

    def scan_callback(self, msg: LaserScan):
        if self.robot_pose is None:
            self.get_logger().warn("尚未取得 /amcl_pose 的位姿資料")
            return

        # 從雷射數據中尋找最短距離（在有效範圍內）的測量值
        min_range = float('inf')
        min_index = -1
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and r < min_range:
                min_range = r
                min_index = i
        
        if min_index == -1:
            self.get_logger().warn("未取得有效雷射測距資料")
            return

        # 計算該測距值對應的角度（以雷射座標系）
        angle = msg.angle_min + min_index * msg.angle_increment

        # 將極座標轉換為雷射(機器人)座標的直角座標值
        x_local = min_range * math.cos(angle)
        y_local = min_range * math.sin(angle)

        # 從 /amcl_pose 取得機器人在 world (map) 座標下的位置與朝向
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = quaternion_to_yaw(self.robot_pose.orientation)

        # 將雷射點從機器人座標轉換到世界座標
        global_x = robot_x + (x_local * math.cos(robot_yaw) - y_local * math.sin(robot_yaw))
        global_y = robot_y + (x_local * math.sin(robot_yaw) + y_local * math.cos(robot_yaw))

        self.get_logger().info(
            f"最近牆壁的世界座標: x: {global_x:.2f}, y: {global_y:.2f}, z: 0.0"
        )

def main(args=None):
    rclpy.init(args=args)
    node = NearestWallFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式手動中斷，結束執行")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
