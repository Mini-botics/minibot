import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')
        self.get_logger().info("Distance node has started")

        # Subscriptions
        self.subscription = self.create_subscription(
            String,
            'distance',
            self.distance_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.subscription  # prevent unused variable warning
        self.last_msg = None

    def distance_callback(self, msg):
        distance = float(msg.data)
        self.calculate_velocity(distance)

    def calculate_velocity(self, distance):
        if distance < 15:
            # Turn the robot around to the right
            if self.last_msg != "right":
                self.publish_velocity(0.0, -.5)
                self.get_logger().info("Turning right.")
                self.last_msg = "right"
        else:
            # Move the robot forward
            if self.last_msg != "forward":
                self.publish_velocity(.99, 0.0)
                self.get_logger().info("Moving forward.")
                self.last_msg = "forward"

    def publish_velocity(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        # Debugging information
        self.get_logger().info(f'Linear velocity: {twist.linear.x}')
        self.get_logger().info(f'Angular velocity: {twist.angular.z}')

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceNode()
    rclpy.spin(distance_node)
    distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()