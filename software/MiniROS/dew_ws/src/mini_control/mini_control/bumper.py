import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class BumperNode(Node):

    def __init__(self):
        super().__init__('bumper')
        self.subscription = self.create_subscription(
            Bool,
            'bumper_detected',
            self.listener_callback,
            10)
        self.publisher_vel = self.create_publisher(Twist, 'bumper_vel', 10)
        self.publisher_led = self.create_publisher(Bool, 'bumper_led', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        twist = Twist()
        bumper_led_msg = Bool()
        
        if msg.data:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            bumper_led_msg.data = True  # Turn on LED when bumper is detected
        else:
            bumper_led_msg.data = False  # Turn off LED otherwise
        
        self.publisher_vel.publish(twist)
        self.publisher_led.publish(bumper_led_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
