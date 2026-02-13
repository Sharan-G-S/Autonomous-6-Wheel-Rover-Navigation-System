#!/usr/bin/env python3
"""
YOLOv8 Object Detection Node for Autonomous Rover
Author: Sharan G S
Uses ZED 2i camera feed and COCO dataset for real-time object detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics not installed. Install with: pip install ultralytics")
    YOLO = None


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        
        # Initialize YOLO model
        if YOLO is None:
            self.get_logger().error('YOLOv8 not available. Install ultralytics package.')
            return
            
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        
        # Publishers
        self.detection_pub = self.create_publisher(Image, '/detections/image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detections/markers', 10)
        
        # Storage
        self.latest_depth = None
        
        self.get_logger().info('Object Detector Node initialized')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def image_callback(self, msg):
        """Process image and detect objects"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Annotate image
            annotated_image = results[0].plot()
            
            # Publish annotated image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            detection_msg.header = msg.header
            self.detection_pub.publish(detection_msg)
            
            # Create markers for detected objects
            markers = MarkerArray()
            for idx, detection in enumerate(results[0].boxes.data):
                x1, y1, x2, y2, conf, cls = detection.cpu().numpy()
                
                # Get object center
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # Get depth if available
                distance = 0.0
                if self.latest_depth is not None:
                    if 0 <= center_y < self.latest_depth.shape[0] and \
                       0 <= center_x < self.latest_depth.shape[1]:
                        distance = self.latest_depth[center_y, center_x]
                
                # Create marker
                marker = Marker()
                marker.header = msg.header
                marker.header.frame_id = 'camera_link'
                marker.id = idx
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Position (using depth)
                marker.pose.position.x = float(distance)
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Scale
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                
                # Color (red for obstacles)
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                
                # Lifetime
                marker.lifetime.sec = 1
                
                # Add text
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.id = idx + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose = marker.pose
                text_marker.pose.position.z = 0.3
                text_marker.scale.z = 0.1
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.text = f"{self.model.names[int(cls)]} ({conf:.2f})"
                text_marker.lifetime.sec = 1
                
                markers.markers.append(marker)
                markers.markers.append(text_marker)
            
            self.marker_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
