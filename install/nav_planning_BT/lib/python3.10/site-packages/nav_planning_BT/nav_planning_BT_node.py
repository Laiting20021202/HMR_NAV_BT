#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import py_trees

class BasicConditionTestingNode(Node):
    def __init__(self):
        super().__init__('BT_node')
        # 初始狀態值
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
        self.get_logger().info(f"接收到 BC_LV: {self.bc_lv}")

    def mc_callback(self, msg):
        self.mc_lv = msg.data
        self.get_logger().info(f"接收到 MC_LV: {self.mc_lv}")

    def publish_nav_situation(self, value: int):
        """發布 nav_situation topic 的方法"""
        msg = Int32()
        msg.data = value
        self.nav_situation_pub.publish(msg)
        self.get_logger().info(f"Published nav_situation: {value}")

    # getter 提供給行為樹節點存取當前值
    def get_bc_lv_value(self):
        return self.bc_lv

    def get_mc_lv_value(self):
        return self.mc_lv

    # ---------------------------------------------------
    # 定義各個情境行為 (狀態名稱分別為 S0, S1, S2)
    # ---------------------------------------------------
    class Situation0(py_trees.behaviour.Behaviour):
        def __init__(self, get_bc_lv, node, name="S0"):
            super().__init__(name)
            self.get_bc_lv = get_bc_lv
            self.node = node

        def update(self):
            bc_lv = self.get_bc_lv()
            if bc_lv == 3:
                # s0 條件成立，發布 nav_situation = 0
                self.node.publish_nav_situation(0)
                self.logger.info("situation_0 觸發：輕微避障")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Situation1(py_trees.behaviour.Behaviour):
        def __init__(self, get_bc_lv, get_mc_lv, node, name="S1"):
            super().__init__(name)
            self.get_bc_lv = get_bc_lv
            self.get_mc_lv = get_mc_lv
            self.node = node

        def update(self):
            bc_lv = self.get_bc_lv()
            mc_lv = self.get_mc_lv()
            # s1 條件成立：BC_LV為1或(BC_LV為2且MC_LV為3)
            if bc_lv == 1 or (bc_lv == 2 and mc_lv == 3):
                self.node.publish_nav_situation(1)
                self.logger.info("situation_1 觸發：立即靠邊停車")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Situation2(py_trees.behaviour.Behaviour):
        def __init__(self, node, name="S2"):
            super().__init__(name)
            self.node = node

        def update(self):
            # s2 為預設情境，直接發布 nav_situation = 2
            self.node.publish_nav_situation(2)
            self.logger.info("situation_2 觸發：進入指定區域")
            return py_trees.common.Status.SUCCESS

    # ---------------------------------------------------
    # 建立行為樹：依序檢查 S0 → S1 → S2
    # ---------------------------------------------------
    def create_behavior_tree(self):
        s0 = self.Situation0(self.get_bc_lv_value, self, "S0")
        s1 = self.Situation1(self.get_bc_lv_value, self.get_mc_lv_value, self, "S1")
        s2 = self.Situation2(self, "S2")
        # 使用 Selector 組合三個狀態，按順序評估：若 S0 成立則不再檢查 S1 與 S2，
        # 否則依序檢查 S1，最後若皆不成立則進入 s2 狀態。
        situation_selector = py_trees.composites.Selector("SituationSelector", memory=False)
        situation_selector.add_children([s0, s1, s2])
        return py_trees.trees.BehaviourTree(situation_selector)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BasicConditionTestingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
