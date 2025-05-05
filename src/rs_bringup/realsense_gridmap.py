import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

class RealSenseGridMap(Node):
    def __init__(self):
        super().__init__('realsense_grid_map')
        self.bridge = CvBridge()

        self.grid_resolution = 0.05  # 每格為 5cm
        self.grid_size = (100, 100)  # 5m × 5m

        self.fx = self.fy = self.cx = self.cy = None
        self.has_camera_info = False

        self.costmap_data = None
        self.costmap_received = False

        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.has_camera_info = True

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_data = msg
        self.costmap_received = True

    def depth_callback(self, msg: Image):
        if not self.has_camera_info:
            return

        # ----------- STEP 1: RealSense Depth → Grid Mask -----------
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32) / 1000.0  # mm → m

        h, w = depth_array.shape
        grid = np.zeros(self.grid_size, dtype=np.uint8)  # 0: unknown, 1: free, 2: obstacle

        for v in range(0, h, 4):
            for u in range(0, w, 4):
                d = depth_array[v, u]
                if d == 0.0 or np.isnan(d) or np.isinf(d) or d > 5.0:
                    continue

                x = (u - self.cx) * d / self.fx
                y = (v - self.cy) * d / self.fy
                z = d

                if z < 0.3 or z > 2.5:
                    continue

                gx = int((x + 2.5) / self.grid_resolution)
                gy = int(z / self.grid_resolution)

                if 0 <= gx < self.grid_size[1] and 0 <= gy < self.grid_size[0]:
                    grid[gy, gx] = 2

        # ----------- STEP 2: 建立圖像底圖（local costmap） -----------
        img = np.zeros((self.grid_size[0], self.grid_size[1], 3), dtype=np.uint8)

        if self.costmap_received:
            data = np.array(self.costmap_data.data).reshape((self.costmap_data.info.height, self.costmap_data.info.width))
            res = self.costmap_data.info.resolution
            for y in range(min(data.shape[0], self.grid_size[0])):
                for x in range(min(data.shape[1], self.grid_size[1])):
                    v = data[y, x]
                    if v < 0:
                        img[y, x] = (80, 80, 80)      # Unknown
                    elif v < 50:
                        img[y, x] = (255, 255, 255)   # Free
                    else:
                        img[y, x] = (0, 0, 0)         # Obstacle

        # ----------- STEP 3: 疊上 RealSense 障礙物 -----------
        for y in range(self.grid_size[0]):
            for x in range(self.grid_size[1]):
                if grid[y, x] == 2:
                    img[y, x] = (0, 0, 255)  # 紅色障礙

        # 藍框標記
        obstacle_mask = cv2.inRange(img, (0, 0, 254), (0, 0, 255))
        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 10:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)

        # ----------- STEP 4: 視覺調整與顯示 -----------
        img = cv2.resize(img, (500, 500), interpolation=cv2.INTER_NEAREST)
        img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.flip(img, 1)

        cv2.imshow("RealSense + Local Costmap", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseGridMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
