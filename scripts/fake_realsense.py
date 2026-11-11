#!/usr/bin/env python3
# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>
"""Simulated wrist RealSense for testing cloud_filter_node without hardware.

The camera looks down at a table (z = 0) with an object on it. The raw cloud
also contains what each filter stage of node 1 has to remove:

- the gripper fingers right in front of the lens  -> depth crop (min_depth)
- the floor around the table, 75 cm below it      -> depth crop (max_depth)
- isolated flying pixels in the field of view     -> statistical outlier removal
- a cable hanging between camera and table        -> largest cluster
- invalid (NaN) pixels                            -> dropped by the crop

The whole scene can be moved with scene_offset ([x, y, z] in world_frame).
With publish_tf:=true the world -> camera TF is broadcast from cam_x/y/z, which
can be changed at runtime (ros2 param set), so the simulator also drives node 2
through node 1. With publish_tf:=false the camera pose is read from TF (e.g. the
robot's wrist), so the scene stays fixed in world_frame while the robot moves.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def solid(n, rgb):
    return np.tile(np.array(rgb, dtype=np.uint8), (n, 1))


def pack_rgb(rgb):
    """(N, 3) uint8 -> (N,) float32 with 0x00RRGGBB bits, as in the RealSense cloud."""
    rgb = rgb.astype(np.uint32)
    return ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).view(np.float32)


FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(('x', 'y', 'z', 'rgb'))]


def grid(x0, x1, y0, y1, res):
    x, y = np.meshgrid(np.arange(x0, x1, res), np.arange(y0, y1, res))
    return x.ravel(), y.ravel()


class FakeRealSense(Node):

    def __init__(self):
        super().__init__('fake_realsense')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'wrist_depth_optical_frame')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cam_x', 0.0)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.5)
        # [m] position of the scene centre (object on the table) in world_frame
        self.declare_parameter('scene_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('h_fov_deg', 87.0)   # D435 depth field of view
        self.declare_parameter('v_fov_deg', 58.0)
        self.declare_parameter('noise_std', 0.001)  # [m] depth noise at 0.5 m, grows with depth^2
        self.declare_parameter('n_flying_pixels', 400)
        self.declare_parameter('invalid_ratio', 0.02)
        self.declare_parameter('rate', 10.0)
        self.rng = np.random.default_rng(0)

        # Static scene in the world frame
        x, y = grid(-0.4, 0.4, -0.3, 0.3, 0.002)
        z = 0.06 * np.exp(-((x - 0.05) ** 2 / 0.006 + (y + 0.02) ** 2 / 0.004))  # object on the table
        table = np.stack([x, y, z], axis=1)
        # Wooden table, the object goes from orange to red with its height
        h = np.clip(z / 0.06, 0.0, 1.0)[:, None]
        on_object = z > 0.002
        table_rgb = np.where(on_object[:, None],
                             (1.0 - h) * [255, 160, 40] + h * [220, 30, 30],
                             [160, 120, 80]).astype(np.uint8)

        x, y = grid(-1.2, 1.2, -1.0, 1.0, 0.008)
        outside = (np.abs(x) > 0.42) | (np.abs(y) > 0.32)
        floor = np.stack([x[outside], y[outside], np.full(outside.sum(), -0.75)], axis=1)
        floor_rgb = solid(len(floor), [90, 90, 100])

        # Cable: a 1 cm thick tube hanging 15-20 cm above the table
        t = self.rng.uniform(0.0, 1.0, 3000)
        a = np.array([-0.20, 0.10, 0.15])
        b = np.array([-0.02, 0.16, 0.20])
        cable = a + t[:, None] * (b - a) + self.rng.normal(0.0, 0.003, (len(t), 3))

        cable_rgb = solid(len(cable), [20, 20, 20])

        self.scene = np.vstack([table, floor, cable])
        self.scene_rgb = np.vstack([table_rgb, floor_rgb, cable_rgb])

        # Gripper fingers, fixed in the camera frame, 5-7 cm in front of the lens
        fx, fy = grid(-0.01, 0.01, 0.02, 0.03, 0.001)
        fingers = [np.stack([fx + dx, fy, np.full(fx.shape, 0.06)], axis=1) for dx in (-0.03, 0.03)]
        self.fingers = np.vstack(fingers)
        self.fingers_rgb = solid(len(self.fingers), [200, 200, 210])

        self.pub = self.create_publisher(PointCloud2, '/wrist/depth/color/points', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)
        self.get_logger().info(
            'Publishing on %s, frame %s' % (self.pub.topic_name, self.get_parameter('camera_frame').value))

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        world_frame = self.get_parameter('world_frame').value
        camera_frame = self.get_parameter('camera_frame').value

        if self.get_parameter('publish_tf').value:
            # Optical frame looking down: rotation of 180 deg about world X
            t = np.array([self.get_parameter(n).value for n in ('cam_x', 'cam_y', 'cam_z')], dtype=float)
            R = np.diag([1.0, -1.0, -1.0])
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = world_frame
            tf.child_frame_id = camera_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t
            tf.transform.rotation.x = 1.0
            tf.transform.rotation.w = 0.0
            self.tf_broadcaster.sendTransform(tf)
        else:
            # Camera pose comes from someone else (e.g. robot_state_publisher)
            try:
                tf = self.tf_buffer.lookup_transform(world_frame, camera_frame, Time())
            except TransformException as e:
                self.get_logger().warn('Waiting for TF %s -> %s: %s' % (world_frame, camera_frame, e),
                                       throttle_duration_sec=2.0)
                return
            tr, q = tf.transform.translation, tf.transform.rotation
            t = np.array([tr.x, tr.y, tr.z])
            R = quat_to_matrix(q.x, q.y, q.z, q.w)
            # Stamp the cloud with the pose it was rendered from
            if tf.header.stamp.sec or tf.header.stamp.nanosec:
                stamp = tf.header.stamp

        # World-fixed scene in camera frame: p_cam = R^T (p_world - t)
        offset = np.array(self.get_parameter('scene_offset').value, dtype=float)
        p = (self.scene + offset - t) @ R
        p = np.vstack([p, self.fingers])
        rgb = np.vstack([self.scene_rgb, self.fingers_rgb])

        tan_h = math.tan(math.radians(self.get_parameter('h_fov_deg').value) / 2.0)
        tan_v = math.tan(math.radians(self.get_parameter('v_fov_deg').value) / 2.0)
        visible = (p[:, 2] > 0.03) & (np.abs(p[:, 0]) < p[:, 2] * tan_h) & \
                  (np.abs(p[:, 1]) < p[:, 2] * tan_v)
        p = p[visible]
        rgb = rgb[visible]

        # Depth noise along the viewing ray, growing with the square of the distance
        sigma = self.get_parameter('noise_std').value * (p[:, 2] / 0.5) ** 2
        p *= (1.0 + self.rng.normal(0.0, 1.0, len(p)) * sigma / p[:, 2])[:, None]

        # Flying pixels, scattered in the field of view between 0.2 and 1.2 m
        n = self.get_parameter('n_flying_pixels').value
        d = self.rng.uniform(0.2, 1.2, n)
        flying = np.stack([self.rng.uniform(-tan_h, tan_h, n) * d,
                           self.rng.uniform(-tan_v, tan_v, n) * d, d], axis=1)
        p = np.vstack([p, flying])
        rgb = np.vstack([rgb, self.rng.integers(0, 256, (n, 3), dtype=np.uint8)])

        # Invalid depth
        invalid = self.rng.uniform(0.0, 1.0, len(p)) < self.get_parameter('invalid_ratio').value
        p[invalid] = np.nan

        header = Header(stamp=stamp, frame_id=camera_frame)
        points = np.column_stack([p.astype(np.float32), pack_rgb(rgb)])
        self.pub.publish(point_cloud2.create_cloud(header, FIELDS, points))


def main():
    rclpy.init()
    node = FakeRealSense()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
