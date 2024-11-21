import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

class NeoPixelNode(Node):

    def __init__(self):
        super().__init__('neopixel_node')
        self.get_logger().info("Neopixel Node has started")

        self.colors = {
            'Blue': '0,0,255',
            'Red': '255,0,0',
            'Green': '0,255,0',
            'Off': '0,0,0'
        }

        self.bumper_subscription = self.create_subscription(
            Bool,
            'bumper_led',
            self.bumper_callback,
            10)
        self.emergency_subscription = self.create_subscription(
            Bool,
            'emergency_led',
            self.emergency_callback,
            10)
        self.battery_subscription = self.create_subscription(
            Int32,
            'battery_led',
            self.battery_callback,
            10)
        self.sobe_subscription = self.create_subscription(
            Bool,
            'sobe',
            self.sobe_callback,
            10)
        self.publisher = self.create_publisher(String, 'pixel_color', 18)
        self.battery_brightness_publisher = self.create_publisher(String, 'battery_color', 18)

        # Initialize states
        self.bumper_state = False
        self.emergency_state = False

    def generate_color_string(self, color_name):
        color_value = self.colors[color_name]
        return ','.join([color_value] * 10)

    def generate_battery_color(self, level):
        red = int((1 - level / 10) * 255)
        green = int((level / 10) * 255)   
        return f'{red},{green},0,1.0'

    def bumper_callback(self, msg):
        # Check if the state has changed
        if msg.data != self.bumper_state:
            self.bumper_state = msg.data  # Update the state flag
            if msg.data:
                self.get_logger().info('Bumper detected, publishing Blue color')
                color_string = self.generate_color_string('Blue')
            else:
                self.get_logger().info('No bumper detected, turning off pixel')
                color_string = self.generate_color_string('Off')
            color_msg = String()
            color_msg.data = color_string
            self.publisher.publish(color_msg)

    def emergency_callback(self, msg):
        # Check if the state has changed
        if msg.data != self.emergency_state:
            self.emergency_state = msg.data  # Update the state flag
            if msg.data:
                self.get_logger().info('Emergency LED is on, publishing Red color')
                color_string = self.generate_color_string('Red')
            else:
                self.get_logger().info('Emergency LED is off, turning off pixel')
                color_string = self.generate_color_string('Off')
            color_msg = String()
            color_msg.data = color_string
            self.publisher.publish(color_msg)

    def battery_callback(self, msg):
        level = msg.data
        color_string = self.generate_battery_color(level)
        self.get_logger().info(f'Battery level received: {level}, publishing color: {color_string}')
        color_msg = String()
        color_msg.data = color_string
        self.publisher.publish(color_msg
                               )
    def sobe_callback(self, msg):
        self.get_logger().info(f'Sobe message received: {msg.data}, publishing to sobe_color')
        color_msg = String()
        if msg.data:
            color_msg.data = self.generate_color_string('Blue')  # You can change this to any color you want for "sobe"
        else:
            color_msg.data = self.generate_color_string('Off')
        self.publisher.publish(color_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = NeoPixelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







