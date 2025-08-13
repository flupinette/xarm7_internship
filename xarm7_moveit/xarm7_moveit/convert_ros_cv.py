#! /usr/bin/env python3
"""
This script is a ROS 2 node that processes a single image from a camera topic.
It subscribes to the `/camera/color/image_raw` topic, converts the ROS Image message to an OpenCV image,
and applies Hough Circle detection to find circles in the image. If circles are detected, it draws the first detected circle on the image and displays it.
The node will shut down after processing the first image to avoid multiple detections.
- ros2 run xarm7_moveit convert_ros_cv
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class SingleImageProcessor(Node):
    def __init__(self):
        super().__init__('single_image_processor')
        # Create a subscriber
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Adapt your topic if needed
            self.image_callback,
            10)
        self.processed = False
        self.window_name = "Image"

    def image_callback(self, msg: Image):
        if self.processed:
            return

        encoding = msg.encoding
        height = msg.height
        width = msg.width
        step = msg.step

        try:
            if encoding == 'rgb8':
                channels = 3
            elif encoding == 'bgr8':
                channels = 3
            elif encoding == 'mono8':
                channels = 1
            else:
                self.get_logger().warn(f'Encoding non supporté: {encoding}')
                return

            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            img_2d = np_arr.reshape((height, step))

            img = img_2d[:, :width*channels]

            if channels > 1:
                img = img.reshape((height, width, channels))
            else:
                img = img.reshape((height, width))

            if encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Convert to grayscale and blur
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if channels == 3 else img
            gray = cv2.GaussianBlur(gray, (5,5), 0)

            # Hough Circle Detection
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                       param1=100, param2=30, minRadius=50, maxRadius=300)
            
            if circles is not None:
                print(f"Number of detected circles: {len(circles[0])}")
            else:
                print("No circles detected")
                
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                # Keep only the largest circle
                x,y,r = circles[0]
                # Draw the circle on the frame
                display_img = img.copy()
                cv2.circle(display_img, (x,y), r, (0,255,0), 4)
                cv2.circle(display_img, (x,y), 2, (0,0,255), 3)
                cv2.imshow("Image with Circle", display_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                cv2.imshow(self.window_name, img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            self.processed = True
            self.get_logger().info("Image processed, shutting down node")
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SingleImageProcessor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()