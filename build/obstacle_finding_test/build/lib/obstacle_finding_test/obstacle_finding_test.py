#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSenseObstacleDetector 1.6
============================
新增功能：
* 若估算出的障礙物高度 > 2 m，則忽略該目標，不視為障礙物。

沿用 v 1.5 之設定：
1. disparity 先 ×16 再量化 → 遠端目標 (>2 m) 不會全部掉到 0
2. 地面遮罩改用 ±ground_band 狹帶，而非「<=曲線」全刪
3. 形態學：先 3×3 腐蝕、再 3×3 閉運算 (×2) → 保住細長形狀
4. 最小障礙面積 area_th_px 降至 300
* 6 m 偵測距離
* 距離文字疊加
* 接近 (Δd ≥ 5 cm) 或動態 (光流) → 紅框
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
    # ---------- 初始化 ----------
    def __init__(self) -> None:
        super().__init__('realsense_obstacle_detector')

        # 參數
        self.max_z_m       = 5.0      # 最遠偵測距離 (m)
        self.v_bins        = 128
        self.ground_off    = 10
        self.ground_band   = 3        # 地面 ±band px
        self.area_th_px    = 300      # 最小障礙面積 (px²)
        self.flow_th_pix   = 1.5      # 動態判斷光流 (px)
        self.approach_eps  = 0.05     # Δd ≥ 5 cm → 接近
        self.height_th_m   = 1.0      # ★ 新增：高度門檻 (m)

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

        # 相機 & 狀態
        self.fx = self.fy = self.cx = self.cy = None
        self.baseline  = None
        self.prev_gray = None
        self.prev_dist_map = {}       # (gx,gy) → dist

        self.get_logger().info('RealSenseObstacleDetector node started')

    # ---------- 回呼 ----------
    def callback(self, depth_msg: Image, color_msg: Image, info_msg: CameraInfo):
        # 1. 內參 & 基線
        if self.fx is None:
            K = info_msg.k
            self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
            Tx = info_msg.p[3] / -self.fx if self.fx else 0.0
            self.baseline = abs(Tx) if abs(Tx) > 1e-4 else 0.055
            if abs(Tx) < 1e-4:
                self.get_logger().warn('camera_info 無有效基線，改用預設 baseline=55 mm')
            self.get_logger().info(f'Camera fx={self.fx:.1f}, baseline={self.baseline*1000:.0f} mm')

        # 2. 深度影像→float32(m)
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_m = (depth_raw.astype(np.float32) * 0.001
                   if depth_msg.encoding == '16UC1'
                   else depth_raw.astype(np.float32))
        depth_m[depth_m == 0.0] = np.nan
        z = depth_m.copy()

        # 3. disparity (float32) & 量化
        disp = (self.baseline * self.fx) / z                       # float32 px
        disp_q = np.clip(disp * 16.0, 0, 255).astype(np.uint8)     # 1/16 精度 → 8bit

        H, W = disp_q.shape

        # 4. V-disparity → 地面曲線
        v_disp = np.zeros((H, self.v_bins), np.int32)
        for y in range(H):
            hist = cv2.calcHist([disp_q[y].reshape(-1, 1)], [0],
                                None, [self.v_bins], [0, self.v_bins])
            v_disp[y] = hist.flatten()
        peak = medfilt(np.argmax(v_disp, axis=1), 9)
        ground_curve = np.polyval(
            np.polyfit(np.arange(H), peak, 2), np.arange(H)).astype(np.int32) + self.ground_off

        # 5. 地面遮罩 (狹帶)
        ground_mask = np.zeros_like(disp_q, np.uint8)
        for y in range(H):
            d_thr = ground_curve[y]
            band  = np.abs(disp_q[y] - d_thr) <= self.ground_band
            ground_mask[y, band] = 255

        # 6. 障礙初步遮罩
        obstacle_mask = cv2.bitwise_and(
            cv2.bitwise_not(ground_mask),
            ((disp_q > 0) & (z <= self.max_z_m)).astype(np.uint8) * 255)

        # 7. 形態學：先腐蝕再閉運算
        kernel = np.ones((3, 3), np.uint8)
        obstacle_mask = cv2.erode(obstacle_mask, kernel, iterations=1)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

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

            # 最近距離
            dist = float(np.nanmin(z[y:y + h, x:x + w]))
            if np.isnan(dist):
                continue

            # ★ 估算障礙物高度 (m) ＝ h_pixel × dist / fy
            est_height_m = (h * dist) / self.fy if self.fy else np.inf
            if est_height_m > self.height_th_m:
                # 超過 2 m 視為「可通過」→ 直接跳過
                continue

            # 接近判定：以 32×32 網格 index
            cx, cy = x + w // 2, y + h // 2
            gx, gy = int(cx / 32), int(cy / 32)
            is_approach = False
            if (gx, gy) in self.prev_dist_map:
                if self.prev_dist_map[(gx, gy)] - dist >= self.approach_eps:
                    is_approach = True
            curr_dist_map[(gx, gy)] = dist

            # 動態 (光流)
            if flow is not None:
                roi_flow = flow[y:y + h, x:x + w]
                if np.nanmean(np.hypot(roi_flow[..., 0], roi_flow[..., 1])) > self.flow_th_pix:
                    is_approach = True

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
            poly_msg.polygon = Polygon(points=[
                Point32(x=float(x),       y=float(y),       z=0.0),
                Point32(x=float(x + w),   y=float(y),       z=0.0),
                Point32(x=float(x + w),   y=float(y + h),   z=0.0),
                Point32(x=float(x),       y=float(y + h),   z=0.0)
            ])
            self.pub_poly.publish(poly_msg)

        # 發佈影像
        img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        img_msg.header = color_msg.header
        self.pub_img.publish(img_msg)

        # 更新狀態
        self.prev_gray = gray
        self.prev_dist_map = curr_dist_map


# ---------- main ----------
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
