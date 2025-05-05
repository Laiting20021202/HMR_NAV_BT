import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
import numpy as np
import std_msgs.msg
import math

class RealSensePointCloudPublisher(Node):
    def __init__(self):
        super().__init__('realsense_pointcloud_publisher')
        self.bridge = CvBridge()

        self.fx = self.fy = self.cx = self.cy = None
        self.has_camera_info = False

        self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.pc_pub = self.create_publisher(PointCloud2, '/realsense/obstacles', 10)

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.has_camera_info = True

    def depth_callback(self, msg: Image):
        if not self.has_camera_info:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32) / 1000.0  # mm → m
        h, w = depth_array.shape

        points = []

        for v in range(0, h, 4):
            for u in range(0, w, 4):
                d = depth_array[v, u]
                if d == 0.0 or np.isnan(d) or np.isinf(d) or d > 5.0:
                    continue

                x = (u - self.cx) * d / self.fx
                y = (v - self.cy) * d / self.fy
                z = d

                # Optional: filter ground by height (e.g., ignore points lower than 0.1m)
                if z < 0.3 or z > 2.5:
                    continue

                # Frame: camera_link or base_link (depends on your TF tree)
                points.append([x, y, z])

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  # must match your TF frame used in costmap

        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
