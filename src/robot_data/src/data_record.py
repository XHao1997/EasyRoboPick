#!/usr/bin/env python3
import sys
import os
import threading
import select
import termios
import tty
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from rclpy.time import Time
import datetime

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from dynamixel_sdk_custom_interfaces.srv import GetPosition
from std_msgs.msg import Int32


class SnapshotRecorder(Node):
    def __init__(self):
        super().__init__('snapshot_recorder')

        # -------- Parameters --------
        self.declare_parameter('use_tf', True)  # True 使用 TF, False 使用 PoseStamped 话题
        self.declare_parameter('pose_topic', '/tag_pose')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'tag_pose')
        self.declare_parameter('pose_timeout_sec', 0.5)  # 位姿最大允许时间差（秒）

        self.use_tf = self.get_parameter('use_tf').get_parameter_value().bool_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.pose_timeout_sec = (
            self.get_parameter('pose_timeout_sec').get_parameter_value().double_value
        )

        self.output_npy = datetime.datetime.now().strftime('data/' + 'data_%Y%m%d_%H%M%S.npy')

        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = QoSHistoryPolicy.KEEP_LAST

        # -------- Pose source --------
        self.latest_pose = None
        if not self.use_tf:
            # 直接订阅 PoseStamped
            self.pose_sub = self.create_subscription(
                PoseStamped, self.pose_topic, self._pose_cb, qos_sensor
            )
        else:
            # 使用 TF
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- Joint service client --------
        self.get_pos_client = self.create_client(GetPosition, 'get_position')
        while not self.get_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for get_position service...')
        self.get_logger().info('get_position service is available.')

        self.samples = []   # list[dict]

        # -------- Services --------
        self.snapshot_srv = self.create_service(Trigger, 'take_snapshot', self._take_snapshot_srv)
        self.save_srv = self.create_service(Trigger, 'save_now', self._save_now_srv)

        # -------- 订阅 /record_data --------
        self.record_sub = self.create_subscription(
            Int32,
            '/record_data',
            self._record_data_cb,
            10
        )

        # -------- Keyboard thread (ENTER 采样，q+ENTER 保存退出) --------
        self._kbd_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kbd_thread.start()

        self.get_logger().info(
            f"SnapshotRecorder ready. use_tf={self.use_tf}, pose_topic='{self.pose_topic}', "
            f"frames: {self.camera_frame}->{self.tag_frame}, out='{self.output_npy}'"
        )
        self.get_logger().info(
            "Press <ENTER> to snapshot (async), or call /take_snapshot, "
            "or publish Int32(1) to /record_data. 'q'+ENTER to save & quit."
        )

    # ----------------- Callbacks -----------------
    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def _record_data_cb(self, msg: Int32):
        """收到 /record_data == 1 时触发一次采样（异步）"""
        if msg.data == 1:
            self.get_logger().info("/record_data==1, start async snapshot...")
            self._start_snapshot()
        else:
            self.get_logger().debug(f"/record_data received {msg.data}, ignored.")

    # ----------------- Helpers -----------------
    def _lookup_tf_pose(self):
        """Lookup TF camera_frame -> tag_frame; return PoseStamped or None."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame, Time(), timeout=Duration(seconds=0.2)
            )
            ps = PoseStamped()
            ps.header = tf.header
            ps.header.frame_id = tf.header.frame_id  # camera frame
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            return ps
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

    def _get_current_pose(self):
        """
        获取当前可用的 tag 位姿：
        - use_tf=True 时：用 TF 查询
        - use_tf=False 时：用最近收到的 PoseStamped
        - 若没有位姿或位姿太旧，则返回 None（视为无检测）
        """
        pose_msg = self._lookup_tf_pose() if self.use_tf else self.latest_pose
        if pose_msg is None:
            self.get_logger().debug('No pose_msg available (no detection).')
            return None

        # 检查时间是否过旧
        stamp = pose_msg.header.stamp
        t_pose = float(stamp.sec) + stamp.nanosec * 1e-9
        t_now = self.get_clock().now().nanoseconds * 1e-9
        age = t_now - t_pose

        if age > self.pose_timeout_sec:
            self.get_logger().warning(
                f'Pose too old ({age:.3f}s > {self.pose_timeout_sec}s); treat as no detection.'
            )
            return None

        return pose_msg

    def _start_snapshot(self):
        """
        异步启动一次 snapshot：
        1) 先取当前有效 pose（若无检测则直接返回 False）
        2) 发起 get_position 异步请求
        3) 在 future 的 done 回调里真正保存数据
        """
        pose_msg = self._get_current_pose()
        if pose_msg is None:
            # 没检测到 tag，不记录
            self.get_logger().info('No valid tag detection; snapshot not recorded.')
            return False

        req = GetPosition.Request()
        req.id = -1  # all 6 joints

        future = self.get_pos_client.call_async(req)
        # 把 pose_msg capture 进去
        future.add_done_callback(lambda fut, pose=pose_msg: self._on_get_position_done(fut, pose))

        return True

    def _on_get_position_done(self, future, pose_msg: PoseStamped):
        """get_position service 返回后的回调（仍然在 executor 线程里执行）"""
        if future.cancelled():
            self.get_logger().warning('get_position future cancelled.')
            return

        exc = future.exception()
        if exc is not None:
            self.get_logger().warning(f'get_position call failed: {exc}')
            return

        resp = future.result()
        if resp is None or not resp.positions:
            self.get_logger().warning('get_position returned empty.')
            return

        pos = np.array(resp.positions, dtype=float)
        if pos.size != 6:
            self.get_logger().warning(
                f'get_position returned {pos.size} joints (expect 6); snapshot skipped.'
            )
            return

        # ---- pose 已经在 pose_msg 里 ----
        t_now = self.get_clock().now().nanoseconds * 1e-9

        px = pose_msg.pose.position.x
        py = pose_msg.pose.position.y
        pz = pose_msg.pose.position.z

        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        tag_xyz = np.array([px, py, pz], dtype=float)
        tag_quat_xyzw = np.array([qx, qy, qz, qw], dtype=float)  # ROS: x,y,z,w

        sample = {
            't': t_now,
            'joint_positions': pos,
            'tag_xyz': tag_xyz,
            'tag_quat_xyzw': tag_quat_xyzw,
        }

        self.samples.append(sample)
        self.get_logger().info(
            f"Snapshot #{len(self.samples)} captured "
            f"(joints={pos}, xyz={tag_xyz}, quat={tag_quat_xyzw})"
        )

    def _save_to_npy(self):
        if not self.samples:
            self.get_logger().warning('No samples to save.')
            return False
        out = os.path.abspath(self.output_npy)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        np.save(out, np.array(self.samples, dtype=object), allow_pickle=True)
        self.get_logger().info(f"Saved {len(self.samples)} samples to {out}")
        return True

    # ----------------- Services -----------------
    def _take_snapshot_srv(self, req, resp):
        # 注意：这里是“启动异步 snapshot”，而不是等它完成
        ok = self._start_snapshot()
        resp.success = bool(ok)
        resp.message = f"snapshot requested (async), current saved={len(self.samples)}"
        return resp

    def _save_now_srv(self, req, resp):
        ok = self._save_to_npy()
        resp.success = bool(ok)
        resp.message = f"snapshots={len(self.samples)}"
        return resp

    # ----------------- Keyboard loop -----------------
    def _keyboard_loop(self):
        if not sys.stdin.isatty():
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\n':  # ENTER
                        self.get_logger().info("ENTER pressed, start async snapshot...")
                        self._start_snapshot()
                    elif ch.lower() == 'q':
                        self.get_logger().info("Quit by user. Saving to file...")
                        self._save_to_npy()
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._save_to_npy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
