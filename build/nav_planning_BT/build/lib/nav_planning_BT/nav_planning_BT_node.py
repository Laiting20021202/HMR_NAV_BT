#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import py_trees
import py_trees.display

# Global node for publishing
g_node = None

class BasicConditionTestingNode(Node):
    def __init__(self):
        super().__init__('basic_condition_testing_node')
        self.bc_lv = 3
        self.mc_lv = -1

        # Subscriptions
        self.create_subscription(Int32, 'BC_LV', self.bc_callback, 10)
        self.create_subscription(Int32, 'MC_LV', self.mc_callback, 10)
        self.get_logger().info("Subscribed to BC_LV and MC_LV topics.")

        # Publisher
        self.nav_situation_pub = self.create_publisher(Int32, 'nav_situation', 10)

        # Build and setup tree
        self.tree = self.create_behavior_tree()
        self.tree.setup(timeout=15)
        py_trees.display.render_dot_tree(self.tree.root, target_directory='.', name='behavior_tree')

        # Tick at 2Hz for faster reaction
        self.create_timer(0.5, self.tick_tree)

    def bc_callback(self, msg: Int32):
        self.bc_lv = msg.data

    def mc_callback(self, msg: Int32):
        self.mc_lv = msg.data

    def publish_nav_situation(self, value: int):
        msg = Int32()
        msg.data = value
        self.nav_situation_pub.publish(msg)
        self.get_logger().info(f"Published nav_situation: {value}")

    def get_bc_lv_value(self) -> int:
        return self.bc_lv

    def get_mc_lv_value(self) -> int:
        return self.mc_lv

    # Leaf behaviors
    class IsM(py_trees.behaviour.Behaviour):
        def __init__(self, mc_getter, expected, name):
            super().__init__(name)
            self.mc_getter = mc_getter
            self.expected = expected
        def update(self):
            return py_trees.common.Status.SUCCESS if self.mc_getter() == self.expected else py_trees.common.Status.FAILURE

    class IsB(py_trees.behaviour.Behaviour):
        def __init__(self, bc_getter, expected, name):
            super().__init__(name)
            self.bc_getter = bc_getter
            self.expected = expected
        def update(self):
            return py_trees.common.Status.SUCCESS if self.bc_getter() == self.expected else py_trees.common.Status.FAILURE

    class SN(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(-1)
            return py_trees.common.Status.SUCCESS

    class S0(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(0)
            return py_trees.common.Status.SUCCESS

    class S1(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(1)
            return py_trees.common.Status.SUCCESS

    class S2(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(2)
            return py_trees.common.Status.SUCCESS

    def create_behavior_tree(self) -> py_trees.trees.BehaviourTree:
        # Root selector
        root = py_trees.composites.Selector("MZCS", memory=False)

        # Mode sequences and BCCS selectors
        seq_M1 = py_trees.composites.Sequence("M1_sequence", memory=False)
        seq_M2 = py_trees.composites.Sequence("M2_sequence", memory=False)
        seq_M3 = py_trees.composites.Sequence("M3_sequence", memory=False)

        sel_M1 = py_trees.composites.Selector("BCCS_M1", memory=False)
        sel_M2 = py_trees.composites.Selector("BCCS_M2", memory=False)
        sel_M3 = py_trees.composites.Selector("BCCS_M3", memory=False)

        # Build M1 branch
        seq_M1.add_children([
            self.IsM(self.get_mc_lv_value, 1, "is_M1"),
            sel_M1
        ])
        m1_b1 = py_trees.composites.Sequence("M1_B1_sequence", memory=False)
        m1_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.SN("SN_M1_B1")
        ])
        m1_b2 = py_trees.composites.Sequence("M1_B2_sequence", memory=False)
        m1_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.SN("SN_M1_B2")
        ])
        m1_b3 = py_trees.composites.Sequence("M1_B3_sequence", memory=False)
        m1_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M1_B3")
        ])
        sel_M1.add_children([m1_b1, m1_b2, m1_b3])

        # Build M2 branch
        seq_M2.add_children([
            self.IsM(self.get_mc_lv_value, 2, "is_M2"),
            sel_M2
        ])
        m2_b1 = py_trees.composites.Sequence("M2_B1_sequence", memory=False)
        m2_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.S1("S1_M2_B1")
        ])
        m2_b2 = py_trees.composites.Sequence("M2_B2_sequence", memory=False)
        m2_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.S2("S2_M2_B2")
        ])
        m2_b3 = py_trees.composites.Sequence("M2_B3_sequence", memory=False)
        m2_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M2_B3")
        ])
        sel_M2.add_children([m2_b1, m2_b2, m2_b3])

        # Build M3 branch
        seq_M3.add_children([
            self.IsM(self.get_mc_lv_value, 3, "is_M3"),
            sel_M3
        ])
        m3_b1 = py_trees.composites.Sequence("M3_B1_sequence", memory=False)
        m3_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.S1("S1_M3_B1")
        ])
        m3_b2 = py_trees.composites.Sequence("M3_B2_sequence", memory=False)
        m3_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.S1("S1_M3_B2")
        ])
        m3_b3 = py_trees.composites.Sequence("M3_B3_sequence", memory=False)
        m3_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M3_B3")
        ])
        sel_M3.add_children([m3_b1, m3_b2, m3_b3])

        # Add branches to root
        root.add_children([seq_M1, seq_M2, seq_M3])
        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BasicConditionTestingNode()
    global g_node
    g_node = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import py_trees
import py_trees.display

# Global node for publishing
g_node = None

class BasicConditionTestingNode(Node):
    def __init__(self):
        super().__init__('basic_condition_testing_node')
        self.bc_lv = 3
        self.mc_lv = -1

        # Subscriptions
        self.create_subscription(Int32, 'BC_LV', self.bc_callback, 10)
        self.create_subscription(Int32, 'MC_LV', self.mc_callback, 10)
        self.get_logger().info("Subscribed to BC_LV and MC_LV topics.")

        # Publisher
        self.nav_situation_pub = self.create_publisher(Int32, 'nav_situation', 10)

        # Build and setup tree
        self.tree = self.create_behavior_tree()
        self.tree.setup(timeout=15)
        py_trees.display.render_dot_tree(self.tree.root, target_directory='.', name='behavior_tree')

        # Tick at 2Hz for faster reaction
        self.create_timer(0.5, self.tick_tree)

    def bc_callback(self, msg: Int32):
        self.bc_lv = msg.data

    def mc_callback(self, msg: Int32):
        self.mc_lv = msg.data

    def publish_nav_situation(self, value: int):
        msg = Int32()
        msg.data = value
        self.nav_situation_pub.publish(msg)
        self.get_logger().info(f"Published nav_situation: {value}")

    def get_bc_lv_value(self) -> int:
        return self.bc_lv

    def get_mc_lv_value(self) -> int:
        return self.mc_lv

    # Leaf behaviors
    class IsM(py_trees.behaviour.Behaviour):
        def __init__(self, mc_getter, expected, name):
            super().__init__(name)
            self.mc_getter = mc_getter
            self.expected = expected
        def update(self):
            return py_trees.common.Status.SUCCESS if self.mc_getter() == self.expected else py_trees.common.Status.FAILURE

    class IsB(py_trees.behaviour.Behaviour):
        def __init__(self, bc_getter, expected, name):
            super().__init__(name)
            self.bc_getter = bc_getter
            self.expected = expected
        def update(self):
            return py_trees.common.Status.SUCCESS if self.bc_getter() == self.expected else py_trees.common.Status.FAILURE

    class SN(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(-1)
            return py_trees.common.Status.SUCCESS

    class S0(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(0)
            return py_trees.common.Status.SUCCESS

    class S1(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(1)
            return py_trees.common.Status.SUCCESS

    class S2(py_trees.behaviour.Behaviour):
        def __init__(self, name):
            super().__init__(name)
        def update(self):
            global g_node
            g_node.publish_nav_situation(2)
            return py_trees.common.Status.SUCCESS

    def create_behavior_tree(self) -> py_trees.trees.BehaviourTree:
        # Root selector
        root = py_trees.composites.Selector("MZCS", memory=False)

        # Mode sequences and BCCS selectors
        seq_M1 = py_trees.composites.Sequence("M1_sequence", memory=False)
        seq_M2 = py_trees.composites.Sequence("M2_sequence", memory=False)
        seq_M3 = py_trees.composites.Sequence("M3_sequence", memory=False)

        sel_M1 = py_trees.composites.Selector("BCCS_M1", memory=False)
        sel_M2 = py_trees.composites.Selector("BCCS_M2", memory=False)
        sel_M3 = py_trees.composites.Selector("BCCS_M3", memory=False)

        # Build M1 branch
        seq_M1.add_children([
            self.IsM(self.get_mc_lv_value, 1, "is_M1"),
            sel_M1
        ])
        m1_b1 = py_trees.composites.Sequence("M1_B1_sequence", memory=False)
        m1_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.SN("SN_M1_B1")
        ])
        m1_b2 = py_trees.composites.Sequence("M1_B2_sequence", memory=False)
        m1_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.SN("SN_M1_B2")
        ])
        m1_b3 = py_trees.composites.Sequence("M1_B3_sequence", memory=False)
        m1_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M1_B3")
        ])
        sel_M1.add_children([m1_b1, m1_b2, m1_b3])

        # Build M2 branch
        seq_M2.add_children([
            self.IsM(self.get_mc_lv_value, 2, "is_M2"),
            sel_M2
        ])
        m2_b1 = py_trees.composites.Sequence("M2_B1_sequence", memory=False)
        m2_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.S1("S1_M2_B1")
        ])
        m2_b2 = py_trees.composites.Sequence("M2_B2_sequence", memory=False)
        m2_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.S2("S2_M2_B2")
        ])
        m2_b3 = py_trees.composites.Sequence("M2_B3_sequence", memory=False)
        m2_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M2_B3")
        ])
        sel_M2.add_children([m2_b1, m2_b2, m2_b3])

        # Build M3 branch
        seq_M3.add_children([
            self.IsM(self.get_mc_lv_value, 3, "is_M3"),
            sel_M3
        ])
        m3_b1 = py_trees.composites.Sequence("M3_B1_sequence", memory=False)
        m3_b1.add_children([
            self.IsB(self.get_bc_lv_value, 1, "is_B1"),
            self.S1("S1_M3_B1")
        ])
        m3_b2 = py_trees.composites.Sequence("M3_B2_sequence", memory=False)
        m3_b2.add_children([
            self.IsB(self.get_bc_lv_value, 2, "is_B2"),
            self.S1("S1_M3_B2")
        ])
        m3_b3 = py_trees.composites.Sequence("M3_B3_sequence", memory=False)
        m3_b3.add_children([
            self.IsB(self.get_bc_lv_value, 3, "is_B3"),
            self.S0("S0_M3_B3")
        ])
        sel_M3.add_children([m3_b1, m3_b2, m3_b3])

        # Add branches to root
        root.add_children([seq_M1, seq_M2, seq_M3])
        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BasicConditionTestingNode()
    global g_node
    g_node = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
