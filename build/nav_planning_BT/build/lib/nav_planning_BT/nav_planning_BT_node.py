#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import py_trees

# 全域變數，待在 main() 指定 node 實例後供各 leaf 使用
g_node = None

class BasicConditionTestingNode(Node):
    def __init__(self):
        super().__init__('basic_condition_testing_node')
        # 初始化數值
        self.bc_lv = 3
        self.mc_lv = -1

        # 訂閱 BC_LV 與 MC_LV topic
        self.create_subscription(Int32, 'BC_LV', self.bc_callback, 10)
        self.create_subscription(Int32, 'MC_LV', self.mc_callback, 10)
        self.get_logger().info("已訂閱 BC_LV 與 MC_LV topic。")

        # 建立 publisher，發布 topic 'nav_situation' (資料型態 Int32)
        self.nav_situation_pub = self.create_publisher(Int32, 'nav_situation', 10)

        # 建立並設定行為樹
        self.tree = self.create_behavior_tree()
        self.tree.setup(timeout=15)

        # 產生 dot 檔案 (可用 Graphviz 轉換成圖片)
        py_trees.display.render_dot_tree(self.tree.root, target_directory='.', name='behavior_tree')

        # 每秒 tick 一次行為樹
        self.create_timer(1.0, self.tick_tree)

    def bc_callback(self, msg):
        self.bc_lv = msg.data
        #self.get_logger().info(f"接收到 BC_LV: {self.bc_lv}")

    def mc_callback(self, msg):
        self.mc_lv = msg.data
        #self.get_logger().info(f"接收到 MC_LV: {self.mc_lv}")

    def publish_nav_situation(self, value: int):
        """發布 nav_situation topic 的方法"""
        msg = Int32()
        msg.data = value
        self.nav_situation_pub.publish(msg)
        self.get_logger().info(f"Published nav_situation: {value}")

    # 提供 getter 給行為樹節點
    def get_bc_lv_value(self):
        return self.bc_lv

    def get_mc_lv_value(self):
        return self.mc_lv

    # -----------------------------
    # 定義 Leaf 節點 (不改變行為樹結構)
    # -----------------------------
    class Prohibited_area(py_trees.behaviour.Behaviour):
        def __init__(self, get_mc_lv, name="PA"):
            super().__init__(name)
            self.get_mc_lv = get_mc_lv

        def update(self):
            mc_lv = self.get_mc_lv()
            if mc_lv == 1:
                global g_node
                g_node.publish_nav_situation(-1)
                self.logger.info("Prohibited_area")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Situation0(py_trees.behaviour.Behaviour):
        def __init__(self, get_bc_lv, name="S0"):
            super().__init__(name)
            self.get_bc_lv = get_bc_lv

        def update(self):
            bc_lv = self.get_bc_lv()
            if bc_lv == 3:
                global g_node
                g_node.publish_nav_situation(0)
                self.logger.info("Situation0 簡單避障")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Situation1(py_trees.behaviour.Behaviour):
        def __init__(self, get_bc_lv, get_mc_lv, name="S1"):
            super().__init__(name)
            self.get_bc_lv = get_bc_lv
            self.get_mc_lv = get_mc_lv

        def update(self):
            bc_lv = self.get_bc_lv()
            mc_lv = self.get_mc_lv()
            if (bc_lv == 1) or (bc_lv == 2 and mc_lv == 3):
                global g_node
                g_node.publish_nav_situation(1)
                self.logger.info("Situation1 立即靠邊停車")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Situation2(py_trees.behaviour.Behaviour):
        def __init__(self, name="S2"):
            super().__init__(name)

        def update(self):
            global g_node
            g_node.publish_nav_situation(2)
            self.logger.info("Situation2 進入指定區域")
            return py_trees.common.Status.SUCCESS

    # -----------------------------
    # 建立 Behavior Tree (不改變原本結構)
    # -----------------------------
    def create_behavior_tree(self):
        s0 = self.Situation0(self.get_bc_lv_value, "S0")
        s1 = self.Situation1(self.get_bc_lv_value, self.get_mc_lv_value, "S1")
        s2 = self.Situation2("S2")
        pa = self.Prohibited_area(self.get_mc_lv_value, "PA")
        # 建立選擇節點，依序檢查 S1 與 S2
        mzcs = py_trees.composites.Selector("MZCS", memory=False)
        mzcs.add_children([s1, s2])
        # 整體行為樹結構，依序檢查 PA、S0 及 mzcs
        bccs = py_trees.composites.Selector("BCCS", memory=False)
        bccs.add_children([pa, s0, mzcs])
        return py_trees.trees.BehaviourTree(bccs)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BasicConditionTestingNode()
    global g_node
    g_node = node  # 指定全域 node 實例，供 leaf 節點呼叫 publish 方法
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
