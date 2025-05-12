#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSenseObstacleDetector 1.4
-----------------------------
* 增加偵測距離上限至 6 m
* 框旁標示每個障礙物距離 (取 ROI 最小深度；單位 m)
* 若同一障礙物距離「持續變近 ≥ 5 cm」，改為紅框；否則綠框
  └ 透過 (cx,cy) 網格化做簡易對應，不依賴外部追蹤器
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.signal import medfilt
from message_filters import ApproximateTimeSynchronizer, Subscriber


class RealSenseObstacleDetector(Node):
    # ----------------- 初始化 -----------------
    def __init__(self) -> None:
        super().__init__('realsense_obstacle_detector')

        # 主要參數
        self.max_z_m     = 6.0       # 偵測深度上限 (m) ← 已加大
        self.v_bins      = 128
        self.ground_off  = 10
        self.area_th_px  = 500
        self.flow_th_pix = 1.5

        self.approach_eps = 0.2     # 前後差 ≥ 5 cm 視為接近

        # ROS I/O
        self.bridge   = CvBridge()
        self.pub_img  = self.create_publisher(Image,          '/obstacles/image_overlay', 10)
        self.pub_poly = self.create_publisher(PolygonStamped, '/obstacles/boxes',         10)

        sub_depth = Subscriber(self, Image,      '/camera/camera/depth/image_rect_raw')
        sub_color = Subscriber(self, Image,      '/camera/camera/color/image_raw')
        sub_info  = Subscriber(self, CameraInfo, '/camera/camera/depth/camera_info')

        self.sync = ApproximateTimeSynchronizer([sub_depth, sub_color, sub_info],
                                                queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        # 相機參數 & 狀態
        self.fx = self.fy = self.cx = self.cy = None
        self.baseline  = None
        self.prev_gray = None
        self.prev_dist_map = {}      # key=(gx,gy) → dist

        self.get_logger().info('RealSenseObstacleDetector node started')

    # ----------------- 同步回呼 -----------------
    def callback(self, depth_msg: Image, color_msg: Image, info_msg: CameraInfo):
        # 1. 相機內參 & 基線
        if self.fx is None:
            K = info_msg.k
            self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
            Tx = info_msg.p[3] / -self.fx if self.fx else 0.0
            self.baseline = abs(Tx) if abs(Tx) > 1e-4 else 0.055
            if abs(Tx) < 1e-4:
                self.get_logger().warn('camera_info 無有效基線，改用預設 baseline=55 mm')
            self.get_logger().info(f'Camera fx={self.fx:.1f}, baseline={self.baseline*1000:.0f} mm')

        # 2. 深度影像 → float32(m)
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_m = (depth_raw.astype(np.float32) * 0.001
                   if depth_msg.encoding == '16UC1'
                   else depth_raw.astype(np.float32))
        depth_m[depth_m == 0.0] = np.nan
        z = depth_m.copy()

        # 3. disparity & V‑disparity
        disp = (self.baseline * self.fx) / z
        disp_u8 = np.nan_to_num(disp).astype(np.uint8)
        H, W = disp_u8.shape

        v_disp = np.zeros((H, self.v_bins), np.int32)
        for y in range(H):
            hist = cv2.calcHist([disp_u8[y].reshape(-1, 1)], [0],
                                None, [self.v_bins], [0, self.v_bins])
            v_disp[y] = hist.flatten()

        peak = medfilt(np.argmax(v_disp, axis=1), 9)
        ground_curve = np.polyval(
            np.polyfit(np.arange(H), peak, 2), np.arange(H)).astype(np.int32) + self.ground_off

        ground_mask = np.zeros_like(disp_u8, np.uint8)
        for y in range(H):
            d_thr = ground_curve[y]
            ground_mask[y, disp_u8[y] <= d_thr] = 255

        obstacle_mask = cv2.bitwise_and(
            cv2.bitwise_not(ground_mask),
            ((disp_u8 > 0) & (z <= self.max_z_m)).astype(np.uint8) * 255)

        obstacle_mask = cv2.morphologyEx(
            obstacle_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), 1)

        num_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(obstacle_mask, 8)

        # 彩影像
        color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        gray      = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)

        # 光流
        flow = (cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                if self.prev_gray is not None else None)

        overlay = color_bgr.copy()
        curr_dist_map = {}

        for i in range(1, num_lbl):
            x, y, w, h, area = stats[i]
            if area < self.area_th_px:
                continue

            roi_depth = z[y:y + h, x:x + w]
            dist = float(np.nanmin(roi_depth))         # 最近距離 (m)
            if np.isnan(dist):                         # 全 NaN → 略過
                continue

            # -------- 接近判定 --------
            cx, cy = x + w // 2, y + h // 2
            gx, gy = int(cx / 32), int(cy / 32)        # 32px 網格化作索引
            is_approach = False
            if (gx, gy) in self.prev_dist_map:
                if self.prev_dist_map[(gx, gy)] - dist >= self.approach_eps:
                    is_approach = True
            curr_dist_map[(gx, gy)] = dist             # 更新 map

            # -------- 動/靜（光流） --------
            if flow is not None:
                fx_roi = flow[y:y + h, x:x + w, 0]
                fy_roi = flow[y:y + h, x:x + w, 1]
                if np.nanmean(np.hypot(fx_roi, fy_roi)) > self.flow_th_pix:
                    is_approach = True                 # 動態亦標紅

            color_box = (0, 0, 255) if is_approach else (0, 255, 0)

            # 畫框 + 距離文字
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color_box, 2)
            label = f'{dist:.2f} m'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(overlay, (x, y - th - 4), (x + tw + 4, y), color_box, -1)
            cv2.putText(overlay, label, (x + 2, y - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

            # PolygonStamped
            poly_msg = PolygonStamped()
            poly_msg.header = color_msg.header
            poly = Polygon(points=[
                Point32(x=float(x),       y=float(y),       z=0.0),
                Point32(x=float(x + w),   y=float(y),       z=0.0),
                Point32(x=float(x + w),   y=float(y + h),   z=0.0),
                Point32(x=float(x),       y=float(y + h),   z=0.0)
            ])
            poly_msg.polygon = poly
            self.pub_poly.publish(poly_msg)

        # 發佈影像
        img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        img_msg.header = color_msg.header
        self.pub_img.publish(img_msg)

        # 更新狀態
        self.prev_gray = gray
        self.prev_dist_map = curr_dist_map


# ----------------- main -----------------
def main() -> None:
    rclpy.init()
    node = RealSenseObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

