import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')
        self.get_logger().info("Battery has started")

        self.subscription = self.create_subscription(
            String,
            'battery_level',
            self.battery_callback,
            10)
        self.publisher_ = self.create_publisher(Int32, 'battery_led', 10)
        self.subscription  # prevent unused variable warning

    def battery_callback(self, msg):
        battery_level = float(msg.data)
        led_brightness = self.calculate_led_brightness(battery_level)
        self.publish_led_brightness(led_brightness)

    def calculate_led_brightness(self, battery_level):
        # Map battery level (0-100) to LED brightness (1-10)
        brightness = max(0, min(10, battery_level // 10))        
        return brightness

    def publish_led_brightness(self, brightness):
        msg = Int32()
        msg.data = brightness
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published LED brightness: {brightness}')

def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryNode()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
