#!/usr/bin/env python3
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class ObstacleMapper(Node):
    """
    Subscribes to Livox pointcloud, performs voxel-grid simplification,
    extracts obstacles, and publishes a simplified voxelized cloud.
    """

    def __init__(self):
        super().__init__("obstacle_mapper")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            "/livox/livox_lidar",
            self.pc_callback,
            qos_profile
        )

        # Publisher
        self.pub = self.create_publisher(
            PointCloud2,
            "/planning/obstacles_voxel",
            qos_profile
        )

        self.voxel_size = 0.25  # meters
        self.ground_z_threshold = -0.3  # ground removal threshold

        self.get_logger().info("[ObstacleMapper] Started.")

    def pc_callback(self, msg: PointCloud2):
        # Convert to numpy array
        points = np.array(list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)))
        if points.shape[0] == 0:
            return

        # Remove ground
        mask = points[:, 2] > self.ground_z_threshold
        points = points[mask]

        # Voxel grid downsampling
        voxel_keys = np.floor(points / self.voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_keys, axis=0, return_index=True)
        voxel_points = points[unique_idx]

        # Publish voxel cloud
        voxel_msg = self.make_pointcloud_msg(voxel_points, msg.header.frame_id)
        self.pub.publish(voxel_msg)

    def make_pointcloud_msg(self, pts, frame):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame

        fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
        ]

        return pc2.create_cloud(header, fields, pts.tolist())


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
