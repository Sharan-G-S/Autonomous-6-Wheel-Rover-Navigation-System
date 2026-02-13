#!/usr/bin/env python3
"""
GPS Goal Converter - Interactive waypoint setter
Author: Sharan G S

Allows setting GPS waypoints via command line or service calls
"""

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
import math


class GPSGoalConverter(Node):
    def __init__(self):
        super().__init__('gps_goal_converter')
        
        self.get_logger().info('GPS Goal Converter ready')
        self.get_logger().info('Enter GPS coordinates to navigate:')
        self.get_logger().info('Format: latitude,longitude')
        self.get_logger().info('Example: 37.7749,-122.4194')
    
    def convert_and_publish(self, lat, lon):
        """Convert GPS to goal and trigger navigation"""
        self.get_logger().info(f'Converting GPS: {lat}, {lon}')
        # This would integrate with the waypoint navigator
        pass


def main(args=None):
    rclpy.init(args=args)
    
    converter = GPSGoalConverter()
    
    # Interactive mode
    print("\nGPS Waypoint Setter")
    print("=" * 50)
    print("Enter GPS coordinates (latitude,longitude) or 'q' to quit")
    print("Example: 37.7749,-122.4194")
    print("=" * 50)
    
    try:
        while rclpy.ok():
            user_input = input("\nEnter GPS coordinates: ").strip()
            
            if user_input.lower() == 'q':
                break
            
            try:
                parts = user_input.split(',')
                if len(parts) != 2:
                    print("Error: Please enter coordinates as: latitude,longitude")
                    continue
                
                lat = float(parts[0].strip())
                lon = float(parts[1].strip())
                
                if not (-90 <= lat <= 90):
                    print("Error: Latitude must be between -90 and 90")
                    continue
                
                if not (-180 <= lon <= 180):
                    print("Error: Longitude must be between -180 and 180")
                    continue
                
                print(f"\nNavigating to: Latitude={lat}, Longitude={lon}")
                print("Launch the gps_waypoint_navigator with these coordinates:")
                print(f"ros2 run rover_gps gps_waypoint_navigator.py --ros-args -p target_latitude:={lat} -p target_longitude:={lon}")
                
            except ValueError:
                print("Error: Invalid number format")
                continue
    
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
