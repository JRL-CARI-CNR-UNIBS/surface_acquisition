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

The world -> camera TF is broadcast too, and the camera position can be changed
at runtime (ros2 param set), so the simulator also drives node 2 through node 1.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


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

        x, y = grid(-1.2, 1.2, -1.0, 1.0, 0.008)
        outside = (np.abs(x) > 0.42) | (np.abs(y) > 0.32)
        floor = np.stack([x[outside], y[outside], np.full(outside.sum(), -0.75)], axis=1)

        # Cable: a 1 cm thick tube hanging 15-20 cm above the table
        t = self.rng.uniform(0.0, 1.0, 3000)
        a = np.array([-0.20, 0.10, 0.15])
        b = np.array([-0.02, 0.16, 0.20])
        cable = a + t[:, None] * (b - a) + self.rng.normal(0.0, 0.003, (len(t), 3))

        self.scene = np.vstack([table, floor, cable])

        # Gripper fingers, fixed in the camera frame, 5-7 cm in front of the lens
        fx, fy = grid(-0.01, 0.01, 0.02, 0.03, 0.001)
        fingers = [np.stack([fx + dx, fy, np.full(fx.shape, 0.06)], axis=1) for dx in (-0.03, 0.03)]
        self.fingers = np.vstack(fingers)

        self.pub = self.create_publisher(PointCloud2, '/wrist/depth/color/points', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)
        self.get_logger().info(
            'Publishing on %s, frame %s' % (self.pub.topic_name, self.get_parameter('camera_frame').value))

    def publish(self):
        cx, cy, cz = (self.get_parameter(n).value for n in ('cam_x', 'cam_y', 'cam_z'))
        stamp = self.get_clock().now().to_msg()
        camera_frame = self.get_parameter('camera_frame').value

        # Optical frame looking down: rotation of 180 deg about world X
        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.get_parameter('world_frame').value
            tf.child_frame_id = camera_frame
            tf.transform.translation.x = cx
            tf.transform.translation.y = cy
            tf.transform.translation.z = cz
            tf.transform.rotation.x = 1.0
            tf.transform.rotation.w = 0.0
            self.tf_broadcaster.sendTransform(tf)

        # Scene in camera frame: (x, y, z) -> (x - cx, -(y - cy), cz - z)
        p = self.scene - np.array([cx, cy, cz])
        p[:, 1] *= -1.0
        p[:, 2] *= -1.0
        p = np.vstack([p, self.fingers])

        tan_h = math.tan(math.radians(self.get_parameter('h_fov_deg').value) / 2.0)
        tan_v = math.tan(math.radians(self.get_parameter('v_fov_deg').value) / 2.0)
        visible = (p[:, 2] > 0.03) & (np.abs(p[:, 0]) < p[:, 2] * tan_h) & \
                  (np.abs(p[:, 1]) < p[:, 2] * tan_v)
        p = p[visible]

        # Depth noise along the viewing ray, growing with the square of the distance
        sigma = self.get_parameter('noise_std').value * (p[:, 2] / 0.5) ** 2
        p *= (1.0 + self.rng.normal(0.0, 1.0, len(p)) * sigma / p[:, 2])[:, None]

        # Flying pixels, scattered in the field of view between 0.2 and 1.2 m
        n = self.get_parameter('n_flying_pixels').value
        d = self.rng.uniform(0.2, 1.2, n)
        flying = np.stack([self.rng.uniform(-tan_h, tan_h, n) * d,
                           self.rng.uniform(-tan_v, tan_v, n) * d, d], axis=1)
        p = np.vstack([p, flying])

        # Invalid depth
        invalid = self.rng.uniform(0.0, 1.0, len(p)) < self.get_parameter('invalid_ratio').value
        p[invalid] = np.nan

        header = Header(stamp=stamp, frame_id=camera_frame)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, p.astype(np.float32)))


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
