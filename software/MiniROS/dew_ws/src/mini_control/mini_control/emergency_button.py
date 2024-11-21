import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class EmergencyLedNode(Node):

    def __init__(self):
        super().__init__('emergency_button')
        self.get_logger().info("Emergency Node has started")

        self.subscription = self.create_subscription(
            Bool,
            'isButtonPressed',
            self.listener_callback,
            10)
        self.publisher_led = self.create_publisher(Bool, 'emergency_led', 10)
        self.publisher_vel = self.create_publisher(Twist, 'emergency_vel', 10)

        self.timer = None  # Timer to continuously publish zero velocity
        self.is_button_pressed = False

    def listener_callback(self, msg):
        emergency_led_msg = Bool()
        
        if msg.data and not self.is_button_pressed:
            self.get_logger().info('Button is pressed, publishing emergency_led = true and zero velocity')
            emergency_led_msg.data = True
            self.is_button_pressed = True

            # Start a timer to continuously publish zero velocity
            self.timer = self.create_timer(0.1, self.publish_zero_velocity)
        
        elif not msg.data and self.is_button_pressed:
            self.get_logger().info('Button is not pressed, turning off emergency_led')
            emergency_led_msg.data = False
            self.is_button_pressed = False

            # Stop the timer
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None

        self.publisher_led.publish(emergency_led_msg)

    def publish_zero_velocity(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyLedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist

# class EmergencyLedNode(Node):

#     def __init__(self):
#         super().__init__('emergency_button')
#         self.get_logger().info("Emergency Node has started")

#         self.subscription = self.create_subscription(
#             Bool,
#             'isButtonPressed',
#             self.listener_callback,
#             10)
#         self.publisher_led = self.create_publisher(Bool, 'emergency_led', 10)
#         self.publisher_vel = self.create_publisher(Twist, 'emergency_vel', 10)

#     def listener_callback(self, msg):
#         emergency_led_msg = Bool()
#         twist = Twist()
        
#         if msg.data:
#             self.get_logger().info('Button is pressed, publishing emergency_led = true and zero velocity')
#             emergency_led_msg.data = True
            
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.publisher_vel.publish(twist)

#         else:
#             self.get_logger().info('Button is not pressed, turning off emergency_led')
#             emergency_led_msg.data = False

#         self.publisher_led.publish(emergency_led_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EmergencyLedNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
