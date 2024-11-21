import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2
import cv2
import numpy as np
import os
import yaml

class ImagePublisher(Node):
    def __init__(self, camera_params):
        super().__init__('image_publisher_node')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image1/compressed', 10)
        self.timer = self.create_timer(camera_params['timer'], self.timer_callback)
        self.picam2 = Picamera2()

        # 0.2, 640,480 80

        config = self.picam2.create_preview_configuration(main={"size": tuple(camera_params['size'])})  # Example
        self.picam2.configure(config)
        self.picam2.start()
        self.jpeg_quality = camera_params['jpeg_quality']

        self.get_logger().info('CSI camera started successfully.')

    def timer_callback(self):
        frame = self.picam2.capture_array()
        if frame is not None:
            # Convert the frame from BGRA to BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # Compress the image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]  # Adjust the quality as needed
            result, encimg = cv2.imencode('.jpeg', frame_bgr, encode_param)

            if result:
                # Create a CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(encimg).tobytes()
                self.publisher_.publish(msg)
                # self.get_logger().info('Publishing compressed image')
            else:
                self.get_logger().warn('Failed to compress image')
        else:
            self.get_logger().warn('Failed to capture image')

def main(args=None):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    parent_dir = os.path.dirname(current_dir)
    config_path = os.path.join(parent_dir, 'config', 'camera_params.yaml')
    with open(config_path, 'r') as file:
        camera_params = yaml.safe_load(file)
    rclpy.init(args=args)
    image_publisher = ImagePublisher(camera_params)
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
