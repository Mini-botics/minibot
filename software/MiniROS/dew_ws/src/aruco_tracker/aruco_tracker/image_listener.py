import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.get_logger().info('Listener node has started')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image1/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Convert the compressed image to a CV2 image
        self.get_logger().info('Receiving image')
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow("Received Image", image_np)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_listener = ImageListener()

    rclpy.spin(image_listener)

    image_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
