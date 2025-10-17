#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import pyttsx3
import threading
import sensor_msgs_py.point_cloud2 as pc2

class YOLOObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('yolo_obstacle_avoidance')
        
        # Parameters
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('obstacle_distance_threshold', 1.0)  # meters
        self.declare_parameter('safe_distance', 0.5)  # meters
        
        model_path = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        
        # Initialize YOLO
        self.yolo_model = YOLO(model_path)
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.rgb_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/depth0/image_raw',
            self.depth_callback,
            10)
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/detections/image', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.latest_depth_image = None
        self.obstacles = []
        
        self.get_logger().info('YOLO Obstacle Avoidance Node Started!')
        self.get_logger().info(f'Using model: {model_path}')

    def depth_callback(self, msg):
        """Store latest depth image for distance calculations"""
        try:
            # Convert depth image to numpy array
            # Depth image encoding is usually 32FC1 (32-bit float, 1 channel) in meters
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f'Depth image shape: {self.latest_depth_image.shape}, dtype: {self.latest_depth_image.dtype}', 
                                  throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')

    def get_distance_at_pixel(self, x, y):
        """Get distance from depth image at pixel coordinates"""
        if self.latest_depth_image is None:
            return None
        
        try:
            height, width = self.latest_depth_image.shape[:2]
            
            # Check bounds
            if x >= width or y >= height or x < 0 or y < 0:
                return None
            
            # Get depth value at pixel
            depth = self.latest_depth_image[int(y), int(x)]
            
            # Handle different depth encodings
            # Some cameras publish depth in mm, some in meters
            # Check if value seems reasonable
            if np.isnan(depth) or depth <= 0:
                return None
            
            # If depth seems to be in mm (values > 100), convert to meters
            if depth > 100:
                depth = depth / 1000.0
            
            # Sanity check: reject unreasonable distances
            if depth > 10.0:  # More than 10 meters seems unreasonable for HP60C
                return None
            
            return float(depth)
            
        except Exception as e:
            self.get_logger().error(f'Error getting distance: {str(e)}')
            return None

    def rgb_callback(self, msg):
        """Process RGB image with YOLO and calculate obstacle distances"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.yolo_model(cv_image, conf=self.confidence_threshold)
            
            # Process detections
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            self.obstacles = []
            
            for result in results:
                boxes = result.boxes
                
                for box in boxes:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.yolo_model.names[class_id]
                    
                    # Calculate center of bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # Get distance to object
                    distance = self.get_distance_at_pixel(center_x, center_y)
                    
                    # Create detection message
                    detection = Detection2D()
                    detection.bbox.center.position.x = float(center_x)
                    detection.bbox.center.position.y = float(center_y)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(class_id)
                    hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis)
                    
                    detection_array.detections.append(detection)
                    
                    # Store obstacle information
                    if distance is not None and distance < self.obstacle_threshold:
                        self.obstacles.append({
                            'class': class_name,
                            'distance': distance,
                            'x': center_x,
                            'confidence': confidence
                        })
                    
                    # Draw on image
                    color = (0, 255, 0) if distance and distance > self.safe_distance else (0, 0, 255)
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    
                    label = f'{class_name} {confidence:.2f}'
                    if distance:
                        label += f' {distance:.2f}m'
                    
                    cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.annotated_image_pub.publish(annotated_msg)
            
            # Calculate and publish navigation command
            self.publish_navigation_command()
            
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {str(e)}')

    def publish_navigation_command(self):
        """Generate navigation commands based on detected obstacles"""
        cmd = Twist()
        
        if not self.obstacles:
            # No obstacles - safe to move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.get_logger().info('Clear path - moving forward', throttle_duration_sec=1.0)
        else:
            # Find closest obstacle
            closest = min(self.obstacles, key=lambda x: x['distance'])
            
            if closest['distance'] < self.safe_distance:
                # Too close - stop or back up
                cmd.linear.x = -0.1
                cmd.angular.z = 0.0
                self.get_logger().warn(
                    f"DANGER! {closest['class']} at {closest['distance']:.2f}m - backing up",
                    throttle_duration_sec=1.0)
            else:
                # Obstacle detected but at safe distance
                # Turn away from obstacle
                image_center = 320  # Assume 640px width
                if closest['x'] < image_center:
                    # Obstacle on left - turn right
                    cmd.angular.z = -0.3
                else:
                    # Obstacle on right - turn left
                    cmd.angular.z = 0.3
                
                cmd.linear.x = 0.1
                self.get_logger().info(
                    f"{closest['class']} at {closest['distance']:.2f}m - avoiding",
                    throttle_duration_sec=1.0)
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()