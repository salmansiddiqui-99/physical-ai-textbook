#!/usr/bin/env python3
"""
Isaac ROS Stereo Depth Processing Pipeline

Purpose: Process stereo images to generate depth maps and point clouds for obstacle avoidance
Environment: Isaac ROS with stereo camera (Isaac Sim or physical RealSense/ZED)
Dependencies: isaac_ros_stereo_image_proc, isaac_ros_nitros

Usage:
    ros2 run isaac_ros_examples depth_pipeline.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from stereo_msgs.msg import DisparityImage
import numpy as np
from cv_bridge import CvBridge
import cv2


class DepthPipelineProcessor(Node):
    """Process depth data from Isaac ROS stereo pipeline for humanoid navigation"""

    def __init__(self):
        super().__init__('depth_pipeline_processor')

        # CV Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Subscribers
        self.disparity_sub = self.create_subscription(
            DisparityImage,
            '/stereo/disparity',
            self.disparity_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/stereo/depth',
            self.depth_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/stereo/points2',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.obstacle_map_pub = self.create_publisher(
            Image,
            '/obstacle_map',
            10
        )

        # Parameters
        self.declare_parameter('min_depth', 0.3)  # Minimum detection distance (m)
        self.declare_parameter('max_depth', 5.0)  # Maximum detection distance (m)
        self.declare_parameter('obstacle_threshold', 1.5)  # Distance to mark as obstacle (m)

        self.get_logger().info('Depth pipeline processor initialized')

    def disparity_callback(self, msg: DisparityImage):
        """
        Process disparity image (pixel shift between left/right cameras)

        Disparity d = (focal_length * baseline) / depth
        Higher disparity = closer object
        """
        # Convert disparity to numpy array
        disparity = np.array(msg.image.data, dtype=np.float32).reshape(
            msg.image.height, msg.image.width
        )

        # Compute depth from disparity
        # depth = (f * baseline) / disparity
        focal_length = msg.f  # Focal length in pixels
        baseline = msg.t      # Baseline in meters

        # Avoid divide-by-zero
        depth = np.where(
            disparity > 0,
            (focal_length * baseline) / disparity,
            0.0
        )

        # Clamp to valid range
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value
        depth = np.clip(depth, min_depth, max_depth)

        # Create obstacle map (binary: obstacle=255, free=0)
        obstacle_threshold = self.get_parameter('obstacle_threshold').value
        obstacle_map = (depth < obstacle_threshold).astype(np.uint8) * 255

        # Publish obstacle map
        obstacle_msg = self.bridge.cv2_to_imgmsg(obstacle_map, encoding='mono8')
        obstacle_msg.header = msg.header
        self.obstacle_map_pub.publish(obstacle_msg)

        self.get_logger().debug(
            f'Disparity range: {disparity.min():.1f}-{disparity.max():.1f} px, '
            f'Depth range: {depth.min():.2f}-{depth.max():.2f} m'
        )

    def depth_callback(self, msg: Image):
        """
        Process depth image directly (already converted from disparity)

        Useful for obstacle detection and collision avoidance
        """
        # Convert ROS Image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Find closest obstacle
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            min_depth = valid_depths.min()
            avg_depth = valid_depths.mean()

            # Warn if obstacle too close
            if min_depth < 0.5:
                self.get_logger().warn(
                    f'Obstacle detected at {min_depth:.2f}m! Average depth: {avg_depth:.2f}m'
                )
            else:
                self.get_logger().info(
                    f'Closest obstacle: {min_depth:.2f}m, Average: {avg_depth:.2f}m'
                )

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Process 3D point cloud for advanced obstacle detection

        Point cloud provides (x, y, z) coordinates for each pixel
        Useful for:
        - Detecting overhead obstacles (low ceilings, doorframes)
        - Stair detection
        - Terrain analysis
        """
        # Parse point cloud header
        num_points = msg.width * msg.height
        point_step = msg.point_step  # Bytes per point
        row_step = msg.row_step      # Bytes per row

        self.get_logger().info(
            f'Point cloud: {num_points} points '
            f'({msg.width}x{msg.height}), '
            f'{point_step} bytes/point'
        )

        # For full point cloud processing, use sensor_msgs_py.point_cloud2
        # Example:
        # import sensor_msgs_py.point_cloud2 as pc2
        # points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # for x, y, z in points:
        #     # Process each 3D point
        #     pass


def main(args=None):
    rclpy.init(args=args)
    node = DepthPipelineProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
