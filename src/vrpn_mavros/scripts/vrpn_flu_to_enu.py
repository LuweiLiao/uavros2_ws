#!/usr/bin/env python3
"""Forward Motive/VRPN pose/twist to MAVROS without coordinate rotation.

Pose:  m       -> m     -> typically /vrpn_enu/pose
Twist: m/s     -> m/s   -> typically /vrpn_enu/twist
       (angular assumed rad/s)

MAVROS then converts ENU -> NED for ArduPilot
(VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE).
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node


def vec_to_output(x, y, z, scale=1.0):
    """Forward an SI-unit vector without changing its coordinate axes."""
    return x * scale, y * scale, z * scale


def quat_to_output(qx, qy, qz, qw):
    """Forward a quaternion without changing its coordinate axes."""
    return qx, qy, qz, qw


class VrpnFluToEnu(Node):
    """ROS 2 adapter retaining the ROS 1 topic and parameter contract."""

    def __init__(self):
        super().__init__("vrpn_flu_to_enu")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("input_topic", "/pend_h1/pose")
        self.declare_parameter("output_topic", "/vrpn_enu/pose")
        self.declare_parameter("twist_input_topic", "/pend_h1/twist")
        self.declare_parameter("twist_output_topic", "/vrpn_enu/twist")
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("enable_twist", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.pose_in = str(self.get_parameter("input_topic").value)
        self.pose_out = str(self.get_parameter("output_topic").value)
        self.twist_in = str(self.get_parameter("twist_input_topic").value)
        self.twist_out = str(self.get_parameter("twist_output_topic").value)
        self.scale = float(self.get_parameter("scale").value)
        self.enable_twist = bool(self.get_parameter("enable_twist").value)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_out, 20)
        self.twist_pub = (
            self.create_publisher(TwistStamped, self.twist_out, 20)
            if self.enable_twist
            else None
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_in, self.pose_cb, 20
        )
        self.twist_sub = (
            self.create_subscription(TwistStamped, self.twist_in, self.twist_cb, 20)
            if self.enable_twist
            else None
        )

        self.get_logger().info(
            "vrpn_flu_to_enu: %s -> %s (pose), twist=%s (%s -> %s), scale=%g"
            % (
                self.pose_in,
                self.pose_out,
                self.enable_twist,
                self.twist_in,
                self.twist_out,
                self.scale,
            )
        )

    def pose_cb(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        ex, ey, ez = vec_to_output(p.x, p.y, p.z, scale=self.scale)
        ox, oy, oz, ow = quat_to_output(o.x, o.y, o.z, o.w)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = ex
        out.pose.position.y = ey
        out.pose.position.z = ez
        out.pose.orientation.x = ox
        out.pose.orientation.y = oy
        out.pose.orientation.z = oz
        out.pose.orientation.w = ow
        self.pose_pub.publish(out)

    def twist_cb(self, msg):
        lin = msg.twist.linear
        ang = msg.twist.angular
        vx, vy, vz = vec_to_output(lin.x, lin.y, lin.z, scale=self.scale)
        wx, wy, wz = vec_to_output(ang.x, ang.y, ang.z, scale=1.0)
        out = TwistStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.linear.z = vz
        out.twist.angular.x = wx
        out.twist.angular.y = wy
        out.twist.angular.z = wz
        self.twist_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VrpnFluToEnu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
