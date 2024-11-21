import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
from geometry_msgs.msg import Twist

class PicoSender(Node):

    def __init__(self):
        super().__init__('pico_sender')
        self.get_logger().info("PicoSender has started")

        # Declare the 'mode' parameter with a default value
        self.declare_parameter('mode', 'twoWheel')
        self.declare_parameter('udp_port', 5007)

        # Retrieve the mode parameter value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        print(f'Mode: {self.mode}')
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        print(f'UDP Port: {self.udp_port}')
        
        # Subscriptions
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with your ROS2 topic name
            self.cmd_vel_callback,
            10)

        self.subscription_pixel_color = self.create_subscription(
            String,
            'pixel_color',  # Replace with your ROS2 topic name
            self.pixel_color_callback,
            10)
        
        self.subscription_battery_brightness = self.create_subscription(
            String,
            'battery_color',  # New topic subscription
            self.battery_brightness_callback,
            10)
        
        self.subscription_sobe = self.create_subscription(
            Bool,
            'sobe',  # New topic subscription
            self.sobe_callback,
            10)
        
        
        # UDP socket setup
        self.udp_ip = "224.0.0.253"  # Multicast IP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        
        
        # Set the TTL (time-to-live) for the multicast message to 1 (local network)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        if self.mode == 'twoWheel':
            message = f'twoWheel|{linear_x},{angular_z}'
        elif self.mode == 'holonomic':
            message = f'holonomic|{linear_x},{linear_y},{angular_z}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')

    def pixel_color_callback(self, msg):
        message = f'neopixel|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    
    def battery_brightness_callback(self, msg):
        message = f'battery|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    def sobe_callback(self, msg):
        message = f'sobe|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')   

     

def main(args=None):
    rclpy.init(args=args)
    pico_sender = PicoSender()
    rclpy.spin(pico_sender)
    pico_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






































# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import socket
# from geometry_msgs.msg import Twist

# class PicoSender(Node):

#     def __init__(self):
#         super().__init__('pico_sender')
#         self.get_logger().info("Pipe_write has started")
#         # Subscriptions
#         self.subscription_cmd_vel = self.create_subscription(
#             Twist,
#             '/cmd_vel',  # Replace with your ROS2 topic name
#             self.cmd_vel_callback,
#             10)

#         self.subscription_pixel_color = self.create_subscription(
#             String,
#             'pixel_color',  # Replace with your ROS2 topic name
#             self.pixel_color_callback,
#             10)
        
#         # UDP socket setup
#         self.udp_ip = "224.0.0.253"  # Multicast IP
#         self.udp_port = 5007       # Multicast port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        
#         # Set the TTL (time-to-live) for the multicast message to 1 (local network)
#         self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    
#     def cmd_vel_callback(self, msg):
#         linear_x = msg.linear.x
#         angular_z = msg.angular.z
#         message = f'keyboard|{linear_x},{angular_z}'
    
#         self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
#         # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    
#     def pixel_color_callback(self, msg):
#         message = f'neopixel|{msg.data}'
    
#         self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
#         # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')

# def main(args=None):
#     rclpy.init(args=args)
#     udp_sender = PicoSender()
#     rclpy.spin(udp_sender)
#     udp_sender.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
