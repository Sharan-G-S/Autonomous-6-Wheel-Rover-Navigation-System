#!/usr/bin/env python3
"""
GPS Waypoint Navigator for Autonomous Rover
Author: Sharan G S

Accepts GPS coordinates (latitude, longitude) and navigates the rover
to that location autonomously using Nav2 stack.

Hardware: EMLID REACH M2 GPS connected to Jetson AGX 64GB
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geographic_msgs.msg import GeoPoint
import math


class GPSWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        
        # Parameters
        self.declare_parameter('target_latitude', 0.0)
        self.declare_parameter('target_longitude', 0.0)
        self.declare_parameter('gps_topic', '/emlid/fix')
        self.declare_parameter('use_manual_datum', False)
        self.declare_parameter('datum_latitude', 0.0)
        self.declare_parameter('datum_longitude', 0.0)
        
        self.target_lat = self.get_parameter('target_latitude').value
        self.target_lon = self.get_parameter('target_longitude').value
        gps_topic = self.get_parameter('gps_topic').value
        
        # Current GPS position
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        
        # Datum (reference point for local coordinate conversion)
        self.datum_lat = None
        self.datum_lon = None
        self.datum_set = False
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self.gps_callback, 10)
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for current goal
        self.goal_pub = self.create_publisher(PoseStamped, '/gps_goal', 10)
        
        self.get_logger().info('GPS Waypoint Navigator initialized')
        self.get_logger().info(f'Target: Lat={self.target_lat}, Lon={self.target_lon}')
    
    def gps_callback(self, msg):
        """Process GPS fix messages from EMLID REACH M2"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        
        # Set datum on first GPS fix if not manually set
        if not self.datum_set:
            use_manual = self.get_parameter('use_manual_datum').value
            if use_manual:
                self.datum_lat = self.get_parameter('datum_latitude').value
                self.datum_lon = self.get_parameter('datum_longitude').value
            else:
                self.datum_lat = self.current_lat
                self.datum_lon = self.current_lon
            
            self.datum_set = True
            self.get_logger().info(f'Datum set: Lat={self.datum_lat}, Lon={self.datum_lon}')
            
            # Send navigation goal once datum is set
            if self.target_lat != 0.0 and self.target_lon != 0.0:
                self.navigate_to_gps_coordinate(self.target_lat, self.target_lon)
    
    def gps_to_local(self, lat, lon):
        """
        Convert GPS coordinates to local XY coordinates (meters)
        Uses simple equirectangular projection (good for small distances)
        """
        if not self.datum_set:
            self.get_logger().warn('Datum not set, cannot convert coordinates')
            return None, None
        
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat1 = math.radians(self.datum_lat)
        lon1 = math.radians(self.datum_lon)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        
        # Calculate x, y in meters
        x = R * (lon2 - lon1) * math.cos((lat1 + lat2) / 2)
        y = R * (lat2 - lat1)
        
        return x, y
    
    def navigate_to_gps_coordinate(self, target_lat, target_lon):
        """Convert GPS coordinate to map frame and send navigation goal"""
        # Convert GPS to local coordinates
        x, y = self.gps_to_local(target_lat, target_lon)
        
        if x is None or y is None:
            self.get_logger().error('Failed to convert GPS coordinates')
            return
        
        self.get_logger().info(f'Target position: X={x:.2f}m, Y={y:.2f}m from datum')
        
        # Create PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Orientation (facing target)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        # Publish goal for visualization
        self.goal_pub.publish(goal_pose)
        
        # Send to Nav2
        self.send_nav2_goal(goal_pose)
    
    def send_nav2_goal(self, goal_pose):
        """Send navigation goal to Nav2 action server"""
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info('Sending navigation goal to Nav2...')
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted, rover is moving...')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback during navigation"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Calculate distance to goal
        if self.current_lat and self.current_lon:
            current_x, current_y = self.gps_to_local(self.current_lat, self.current_lon)
            target_x, target_y = self.gps_to_local(self.target_lat, self.target_lon)
            
            if current_x and target_x:
                distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                self.get_logger().info(f'Distance to goal: {distance:.2f}m', throttle_duration_sec=2.0)
    
    def nav_result_callback(self, future):
        """Handle Nav2 navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info('Navigation completed successfully!')
        else:
            self.get_logger().error('Navigation failed')
    
    def set_new_waypoint(self, lat, lon):
        """Set a new GPS waypoint for navigation"""
        self.target_lat = lat
        self.target_lon = lon
        self.get_logger().info(f'New waypoint set: Lat={lat}, Lon={lon}')
        
        if self.datum_set:
            self.navigate_to_gps_coordinate(lat, lon)


def main(args=None):
    rclpy.init(args=args)
    
    navigator = GPSWaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
