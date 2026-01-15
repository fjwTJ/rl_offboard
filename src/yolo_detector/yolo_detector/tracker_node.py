import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.sub = self.create_subscription(PointStamped, '/perception/target_xyz', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel_body', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tf()
        # PID 参数
        self.kx = 1.0  # 前后
        self.ky = 1.0  # 左右
        self.kz = 0.6  # 上下
        self.k_yaw = 0.3
        self.desired_dist = 3  # 目标距离 m

    def _publish_static_tf(self):
        # base_link_frd -> camera_optical_frame
        # Camera position: +0.12m forward, 0.03m left, 0.242m up (FRD => left/up are negative).
        # Rotation: optical (X right, Y down, Z forward) relative to FRD (X forward, Y right, Z down).
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link_frd"
        t.child_frame_id = "x500_depth_0/RealSenseD455/base_link/RealSenseD455/rgbd"
        t.transform.translation.x = 0.12
        t.transform.translation.y = -0.03
        t.transform.translation.z = -0.242
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = 0.5
        self.tf_static_broadcaster.sendTransform(t)

    def callback(self, msg):
        try:
            pt_frd = self.tf_buffer.transform(
                msg, "base_link_frd", timeout=Duration(seconds=0.1)
            )
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return

        X, Y, Z = pt_frd.point.x, pt_frd.point.y, pt_frd.point.z
        yaw_error = math.atan2(Y, X)
        # 前后误差：希望距离固定
        vx = self.kx * (X - self.desired_dist)
        vy = self.ky * Y
        vz = self.kz * Z
        yaw_rate = -self.k_yaw * yaw_error

        # 限幅
        vx = max(min(vx, 1.0), -1.0)
        vy = max(min(vy, 1.0), -1.0)
        vz = max(min(vz, 1.0), -1.0)
        yaw_rate = max(min(yaw_rate, 1.6), -1.6)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.pub.publish(cmd)

        self.get_logger().info(f"cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
        self.get_logger().info(f"FRD XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
