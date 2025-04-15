#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk
from threading import Thread

class ButtonPublisherNode(Node):
    def __init__(self):
        super().__init__('button_publisher_node')
        # 建立兩個 publisher，佇列大小設定為 10
        self.bc_publisher = self.create_publisher(Int32, 'BC_LV', 10)
        self.mc_publisher = self.create_publisher(Int32, 'MC_LV', 10)

    def publish_bc(self, value: int):
        msg = Int32()
        msg.data = value
        self.bc_publisher.publish(msg)
        self.get_logger().info(f'發布 BC_LV: {value}')

    def publish_mc(self, value: int):
        msg = Int32()
        msg.data = value
        self.mc_publisher.publish(msg)
        self.get_logger().info(f'發布 MC_LV: {value}')

def gui_thread(node: ButtonPublisherNode):
    # 建立 tkinter 視窗
    root = tk.Tk()
    root.title("ROS2 UI")

    # 建立 BC_LV 按鈕區塊
    bc_frame = tk.LabelFrame(root, text="BC_LV 控制", padx=10, pady=10)
    bc_frame.pack(side=tk.LEFT, padx=10, pady=10)

    btn_bc1 = tk.Button(bc_frame, text="1", width=10, command=lambda: node.publish_bc(1))
    btn_bc1.pack(padx=5, pady=5)
    btn_bc2 = tk.Button(bc_frame, text="2", width=10, command=lambda: node.publish_bc(2))
    btn_bc2.pack(padx=5, pady=5)
    btn_bc3 = tk.Button(bc_frame, text="3", width=10, command=lambda: node.publish_bc(3))
    btn_bc3.pack(padx=5, pady=5)

    # 建立 MC_LV 按鈕區塊
    mc_frame = tk.LabelFrame(root, text="MC_LV 控制", padx=10, pady=10)
    mc_frame.pack(side=tk.LEFT, padx=10, pady=10)

    btn_mc1 = tk.Button(mc_frame, text="1", width=10, command=lambda: node.publish_mc(1))
    btn_mc1.pack(padx=5, pady=5)
    btn_mc2 = tk.Button(mc_frame, text="2", width=10, command=lambda: node.publish_mc(2))
    btn_mc2.pack(padx=5, pady=5)
    btn_mc3 = tk.Button(mc_frame, text="3", width=10, command=lambda: node.publish_mc(3))
    btn_mc3.pack(padx=5, pady=5)

    # 啟動 tkinter 事件循環
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisherNode()

    # 以另一個執行緒啟動 UI
    gui_thread_obj = Thread(target=gui_thread, args=(node,), daemon=True)
    gui_thread_obj.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
