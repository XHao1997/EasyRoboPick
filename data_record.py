#!/usr/bin/env python3
import sys, os, threading, select, termios, tty
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from rclpy.time import Time
import datetime
# TF 查询
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class SnapshotRecorder(Node):
    def __init__(self):
        super().__init__('snapshot_recorder')

        # -------- Parameters --------
        self.declare_parameter('use_tf', True)  # True tf, False posestamped
        self.declare_parameter('pose_topic', '/tag_pose')
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'tag_pose')

        self.use_tf = self.get_parameter('use_tf').get_parameter_value().bool_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.output_npy = datetime.datetime.now().strftime('data/'+'data_%Y%m%d_%H%M%S.npy')

        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = QoSHistoryPolicy.KEEP_LAST

        # -------- Subscriptions --------
        self.js_sub = self.create_subscription(JointState, self.joint_topic, self._js_cb, 10)

        self.latest_pose = None
        if not self.use_tf:
            self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, qos_sensor)
        else:
            # ✅ 正确：用 Duration，而不是 rclpy.time.Duration
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_js = None
        self.samples = []   # list[dict]

        # -------- Services --------
        self.snapshot_srv = self.create_service(Trigger, 'take_snapshot', self._take_snapshot_srv)
        self.save_srv = self.create_service(Trigger, 'save_now', self._save_now_srv)

        # -------- Keyboard thread (ENTER 采样，q+ENTER 保存退出) --------
        self._kbd_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kbd_thread.start()

        self.get_logger().info(
            f"SnapshotRecorder ready. use_tf={self.use_tf}, pose_topic='{self.pose_topic}', "
            f"frames: {self.camera_frame}->{self.tag_frame}, out='{self.output_npy}'"
        )
        self.get_logger().info("Press <ENTER> to snapshot, or call /take_snapshot. 'q'+ENTER to save & quit.")

    # ----------------- Callbacks -----------------
    def _js_cb(self, msg: JointState):
        self.latest_js = msg

    def _pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

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

    def _make_snapshot(self):
        if self.latest_js is None:
            self.get_logger().warning('No JointState yet; snapshot skipped.')
            return False

        pose_msg = self._lookup_tf_pose() if self.use_tf else self.latest_pose
        if pose_msg is None:
            self.get_logger().warning('No tag pose yet; snapshot skipped.')
            return False

        # pack dictionary
        stamp_js = self.latest_js.header.stamp
        stamp_pose = pose_msg.header.stamp

        sample = {
            't_joint': float(stamp_js.sec) + stamp_js.nanosec * 1e-9,
            't_pose': float(stamp_pose.sec) + stamp_pose.nanosec * 1e-9,
            'joint_names': list(self.latest_js.name),
            'joint_positions': np.array(self.latest_js.position, dtype=float),
            'joint_velocities': np.array(self.latest_js.velocity, dtype=float) if self.latest_js.velocity else None,
            'joint_effort': np.array(self.latest_js.effort, dtype=float) if self.latest_js.effort else None,
            'pose_frame': pose_msg.header.frame_id,
            'pose_position': np.array([
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z], dtype=float),
            'pose_orientation_xyzw': np.array([
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w], dtype=float),
        }
        self.samples.append(sample)
        self.get_logger().info(f"Snapshot #{len(self.samples)} captured.")
        return True

    def _save_to_npy(self):
        if not self.samples:
            self.get_logger().warning('No samples to save.')
            return False
        out = os.path.abspath(self.output_npy)
        np.save(out, np.array(self.samples, dtype=object), allow_pickle=True)
        self.get_logger().info(f"Saved {len(self.samples)} samples to {out}")
        return True

    # ----------------- Services -----------------
    def _take_snapshot_srv(self, req, resp):
        ok = self._make_snapshot()
        resp.success = bool(ok)
        resp.message = f"snapshots={len(self.samples)}"
        return resp

    def _save_now_srv(self, req, resp):
        ok = self._save_to_npy()
        resp.success = bool(ok)
        resp.message = f"snapshots={len(self.samples)}"
        return resp

    # ----------------- Keyboard loop -----------------
    def _keyboard_loop(self):
        # 非阻塞 stdin（raw mode）
        if not sys.stdin.isatty():
            # 在被 launch 重定向时，没有 TTY；跳过键盘监听
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
                        self._make_snapshot()
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
