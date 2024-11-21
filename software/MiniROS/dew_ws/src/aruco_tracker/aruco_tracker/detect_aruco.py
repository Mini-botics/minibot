import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PolygonStamped, Point32
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image1/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(PolygonStamped, 'aruco_marker', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Receiving compressed image')

        # Decompress the image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            self.get_logger().error('Failed to decode the image')
            return

        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.get_logger().info('Converted image to grayscale')

        # Load the ArUco dictionary (6x6 markers, 1000 unique markers)
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        detector = aruco.ArucoDetector(aruco_dict)
        parameters = aruco.DetectorParameters()
        self.get_logger().info('Loaded ArUco dictionary and parameters')

        # Detect markers
        # corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        # Debug information
        if ids is not None:
            self.get_logger().info(f'Detected {len(ids)} markers')

            # Publish each marker's corners
            for i in range(len(ids)):
                marker_corners = PolygonStamped()
                marker_corners.header.stamp = self.get_clock().now().to_msg()
                for corner in corners[i][0]:
                    point = Point32()
                    point.x = float(corner[0])
                    point.y = float(corner[1])
                    point.z = 0.0
                    marker_corners.polygon.points.append(point)
                self.publisher_.publish(marker_corners)
                self.get_logger().info(f'Published marker corners {marker_corners.polygon.points}')
        else:
            self.get_logger().info('No markers detected')

        # Draw detected markers on the original image
        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
            self.get_logger().info('Drew detected markers on the image')

            # Draw corners on the grayscale image
            for corner in corners:
                for point in corner[0]:
                    cv2.circle(gray, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)

        # Display the original image with markers
        cv2.imshow('Aruco Markers', cv_image)

        # Display the grayscale image with corners
        cv2.imshow('Grayscale Image with Corners', gray)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import cv2
# import cv2.aruco as aruco
# import numpy as np

# class ArucoDetector(Node):
#     def __init__(self):
#         super().__init__('aruco_detector')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             '/camera/image1',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('Receiving compressed image')

#         # Decompress the image
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         if cv_image is None:
#             self.get_logger().error('Failed to decode the image')
#             return

#         # Convert image to grayscale
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         self.get_logger().info('Converted image to grayscale')

#         # Load the ArUco dictionary (6x6 markers, 1000 unique markers)
#         aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
#         parameters = aruco.DetectorParameters_create()
#         self.get_logger().info('Loaded ArUco dictionary and parameters')

#         # Detect markers
#         corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#         # Debug information
#         if ids is not None:
#             self.get_logger().info(f'Detected {len(ids)} markers: {ids.flatten()}')
#         else:
#             self.get_logger().info('No markers detected')

#         # Draw detected markers on the original image
#         if ids is not None:
#             cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
#             self.get_logger().info('Drew detected markers on the image')

#             # Draw corners on the grayscale image
#             for corner in corners:
#                 for point in corner[0]:
#                     cv2.circle(gray, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)

#         # Display the original image with markers
#         cv2.imshow('Aruco Markers', cv_image)

#         # Display the grayscale image with corners
#         cv2.imshow('Grayscale Image with Corners', gray)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     aruco_detector = ArucoDetector()
#     rclpy.spin(aruco_detector)
#     aruco_detector.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
