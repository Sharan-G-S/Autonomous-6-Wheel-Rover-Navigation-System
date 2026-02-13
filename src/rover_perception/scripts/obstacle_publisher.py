#!/usr/bin/env python3
"""
Obstacle Publisher Node - Converts YOLO detections to costmap obstacles
Author: Sharan G S
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # Subscribe to detection markers
        self.marker_sub = self.create_subscription(
            MarkerArray, '/detections/markers', self.marker_callback, 10)
        
        # Publish obstacles for costmap
        self.obstacle_pub = self.create_publisher(
            OccupancyGrid, '/detected_obstacles', 10)
        
        self.get_logger().info('Obstacle Publisher Node initialized')
    
    def marker_callback(self, msg):
        """Convert markers to costmap obstacles"""
        # This is a simplified implementation
        # In production, you would convert 3D positions to 2D costmap coordinates
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
