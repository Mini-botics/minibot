import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PolygonStamped
import numpy as np
from rclpy.timer import Timer
from std_msgs.msg import Bool

class FollowAruco(Node):
    def __init__(self):
        super().__init__('follow_aruco')
        self.subscription = self.create_subscription(
            PolygonStamped,
            'aruco_marker',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_aruco', 10)
        self.sobe_publisher = self.create_publisher(Bool, 'sobe', 10)
        self.subscription  # prevent unused variable warning
        self.publisher_  # prevent unused variable warning
        self.sobe_publisher  # prevent unused variable warning

        self.last_marker_time = self.get_clock().now()
        self.no_marker_timeout = 2.0  # seconds without marker before rotating
        self.timer = self.create_timer(0.1, self.check_marker_timeout)

    def listener_callback(self, msg):
        if msg.polygon.points:
            self.last_marker_time = self.get_clock().now()
            self.get_logger().info(f'Received marker corners {msg.polygon.points}')

            # Calculate the center of the marker
            corners = [(point.x, point.y) for point in msg.polygon.points]
            corners = np.array(corners)
            cX, cY = np.mean(corners, axis=0)

            # Calculate the size of the marker (using the distance between two opposite corners)
            size = np.linalg.norm(corners[0] - corners[2])
            
            self.get_logger().info(f'size{size}')

            # Image dimensions (assuming some known width)
            w = 640  # Replace with your camera's width
            self.get_logger().info(f'center: {cX}')

            if cX > w / 2:
                angular_velocity = -0.00005 * (cX - w / 2)
            else:
                angular_velocity = -0.00003 * (cX - w / 2)
            # Calculate angular velocity based on marker center position
            # angular_velocity = -0.000009 * (cX - w / 2)
            # angular_velocity = 0.0 * (cX - w / 2)
            # Calculate linear velocity based on marker size
            reference_size = 350  # Size of the marker at the desired distance
            max_linear_velocity = 0.4  # Maximum linear velocity
            min_linear_velocity = 0.05  # Minimum linear velocity

            if size < reference_size:
                print(f'size: {size}')
                linear_velocity = 0.0004 * (reference_size - size)
                if linear_velocity < 0.075:
                    linear_velocity = 0.075
                self.publish_sobe(False)      

            else:
                linear_velocity = 0.0
                angular_velocity = 0.0
                self.publish_sobe(True)
            if linear_velocity < min_linear_velocity:
                self.get_logger().info(f'I am here')
                linear_velocity = 0.0
                angular_velocity = 0.0 
                self.publish_sobe(True)      
            # Publish the calculated velocities
            self.publish_velocity(linear_velocity, angular_velocity)
        else:
            self.get_logger().info('No marker detected')

    def publish_velocity(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Debugging information
        self.get_logger().info(f'Linear velocity: {twist.linear.x}')
        self.get_logger().info(f'Angular velocity: {twist.angular.z}')

        self.publisher_.publish(twist)

    def publish_sobe(self, state):
        sobe_msg = Bool()
        sobe_msg.data = state
        self.sobe_publisher.publish(sobe_msg)
        self.get_logger().info(f'Published "sobe" message with state: {state}')      

    def check_marker_timeout(self):
        current_time = self.get_clock().now()
        time_since_last_marker = (current_time - self.last_marker_time).nanoseconds/1e9
        if time_since_last_marker > self.no_marker_timeout:
            self.get_logger().info('No marker detected for a while, rotating to search')
            self.publish_velocity(0.0, 0.099)  # Rotate in place
            self.publish_sobe(False)      


def main(args=None):
    rclpy.init(args=args)
    follow_aruco = FollowAruco()
    rclpy.spin(follow_aruco)
    follow_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
