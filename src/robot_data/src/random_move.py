#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from dynamixel_sdk_custom_interfaces.msg import MoveJoint
from dynamixel_sdk_custom_interfaces.msg import SetPosition 

import time
class RandomMoveAndRecord(Node):
    def __init__(self):
        super().__init__('random_move_and_record')

        # 参数：发送频率和关节范围（自己按需要改）
        self.declare_parameter('rate', 0.2)         # Hz：每 2s 一次
        self.declare_parameter('joint_min', -1.5)   # rad
        self.declare_parameter('joint_max',  1.5)   # rad

        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.joint_min = self.get_parameter('joint_min').get_parameter_value().double_value
        self.joint_max = self.get_parameter('joint_max').get_parameter_value().double_value

        # Publisher
        self.move_pub = self.create_publisher(MoveJoint, '/move_joint', 10)
        # self.gripper_pub = self.create_publisher(SetPosition, '/set_position', 10)

        self.record_pub = self.create_publisher(Int32, '/record_data', 10)

        period = 1.0 / self.rate if self.rate > 0.0 else 2.0
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"RandomMoveAndRecord started. rate={self.rate} Hz, "
            f"joint range=({self.joint_min}, {self.joint_max})"
        )

    def _timer_cb(self):
        # 1) 生成 5 个随机关节角
        q = np.array([0.0]*6)
        valid = False
        while not valid:
            q[0] = np.random.uniform(-1, 1, size=1)[0]
            q[1] = np.random.uniform(-1, 0, size=1)[0]
            q[2] = np.random.uniform(-1, 0, size=1)[0]
            q[3] = -np.pi - q[1] - q[2]
            q[4] = 0.0
            q[5] = 3.00
            if abs(q[3]) < 2.5:
                valid = True


        # 2) 发布 /move_joint
        msg = MoveJoint()
        msg.joint_positions = list(q)
        self.move_pub.publish(msg)
        self.get_logger().info(f"Published random /move_joint: {q}")

        # 3) 触发一次 /record_data = 1
        record_msg = Int32()
        record_msg.data = 1
        time.sleep(3)
        self.record_pub.publish(record_msg)
        self.get_logger().info("Published /record_data = 1")

def main(args=None):
    rclpy.init(args=args)
    node = RandomMoveAndRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
