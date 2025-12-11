#!/usr/bin/env python3
import numpy as np
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class CoveragePlanner(Node):
    """
    Global coverage planner using:
    - /odom for robot pose
    - voxel obstacles from /planning/obstacles_voxel
    - B*-style frontier exploration over a global occupancy grid
    """

    def __init__(self):
        super().__init__("coverage_planner")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.sub_obs = self.create_subscription(
            PointCloud2,
            "/planning/obstacles_voxel",
            self.obstacle_callback,
            qos
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos)

        # Map parameters
        self.map_res = 0.3  # meters per cell
        self.map_extent = 100.0  # 100m × 100m global grid
        self.map_size = int(self.map_extent / self.map_res)

        self.grid = np.zeros((self.map_size, self.map_size), dtype=np.uint8)

        # World origin at map center
        self.origin = np.array([self.map_size // 2, self.map_size // 2])

        # Robot pose
        self.rx = 0.0
        self.ry = 0.0
        self.rtheta = 0.0
        self.have_pose = False

        self.get_logger().info("[CoveragePlanner] Ready (with /odom support).")

    # --------------------------------------------------------
    # 1. Update Robot Pose
    # --------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rtheta = self.quaternion_to_yaw(q)

        self.have_pose = True

    def quaternion_to_yaw(self, q):
        # yaw from quaternion (z,w)
        return math.atan2(2.0*(q.w*q.z), 1 - 2*(q.z*q.z))

    # --------------------------------------------------------
    # 2. Update Global Occupancy Grid from Voxelized Obstacles
    # --------------------------------------------------------
    def obstacle_callback(self, msg):
        if not self.have_pose:
            return

        pts = np.array(list(pc2.read_points(msg, field_names=["x","y","z"], skip_nans=True)))
        if pts.shape[0] == 0:
            return

        # Convert LiDAR points from robot frame → global frame
        cos_t = math.cos(self.rtheta)
        sin_t = math.sin(self.rtheta)

        R = np.array([[cos_t, -sin_t],
                      [sin_t,  cos_t]])

        xy = pts[:, :2].T  # (2, N)
        world_xy = (R @ xy).T + np.array([self.rx, self.ry])

        # Fill occupancy grid
        for xw, yw in world_xy:
            gx = int(xw / self.map_res) + self.origin[0]
            gy = int(yw / self.map_res) + self.origin[1]
            if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                self.grid[gx, gy] = 1   # Mark obstacle

        # Run coverage planning
        goal = self.find_frontier_goal()

        if goal is not None:
            self.navigate_to_goal(goal)

    # --------------------------------------------------------
    # 3. Frontier-Based B* Exploration
    # --------------------------------------------------------
    def find_frontier_goal(self):
        """
        Find the next best frontier cell for exploration.
        Frontier = free cell adjacent to unknown or obstacle boundary.
        """

        frontier_list = []

        gx_r = int(self.rx / self.map_res) + self.origin[0]
        gy_r = int(self.ry / self.map_res) + self.origin[1]

        # Search in a wide radius
        R = 200
        x_min = max(gx_r - R, 1)
        x_max = min(gx_r + R, self.map_size - 2)
        y_min = max(gy_r - R, 1)
        y_max = min(gy_r + R, self.map_size - 2)

        for i in range(x_min, x_max):
            for j in range(y_min, y_max):

                # Free cell only
                if self.grid[i, j] != 0:
                    continue

                # Frontier: any neighbor is obstacle (1) or unknown (0 untouched)
                nbr = self.grid[i-1:i+2, j-1:j+2]

                if np.any(nbr == 1):
                    score = self.frontier_heuristic(i, j, gx_r, gy_r)
                    frontier_list.append((score, (i, j)))

        if len(frontier_list) == 0:
            self.get_logger().warn("No frontier found → Coverage completed.")
            return None

        # Highest-scoring frontier
        frontier_list.sort(reverse=True)
        return frontier_list[0][1]

    def frontier_heuristic(self, i, j, gx_r, gy_r):
        """
        B* frontier score:
        - Maximize distance from explored center (encourage coverage expansion)
        - Add small noise to break ties
        """
        dist = math.sqrt((i - gx_r)**2 + (j - gy_r)**2)
        return dist + np.random.uniform(0, 0.2)

    # --------------------------------------------------------
    # 4. Navigate Toward Selected Frontier Goal
    # --------------------------------------------------------
    def navigate_to_goal(self, goal):
        gx, gy = goal

        # Convert grid → world XY
        tx = (gx - self.origin[0]) * self.map_res
        ty = (gy - self.origin[1]) * self.map_res

        dx = tx - self.rx
        dy = ty - self.ry

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.rtheta)

        distance = math.sqrt(dx*dx + dy*dy)

        # Simple proportional control
        cmd = Twist()
        cmd.linear.x = min(0.8, distance * 0.4)
        cmd.angular.z = angle_error * 1.2

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"[CoveragePlanner] Driving to goal (world): ({tx:.2f}, {ty:.2f}), dist={distance:.2f}"
        )

    def normalize_angle(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
