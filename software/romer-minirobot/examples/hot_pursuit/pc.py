from romer_minirobot.robot import MiniRobot
from romer_minirobot.modules import robot, Bool
from pynput import keyboard
import time

# Multicast group details
MULTICAST_GROUP = '224.0.0.252'
MULTICAST_TOPIC_PORT = 5007

class KeyboardControl():

    def __init__(self, robot, x_linear_speed, z_angular_speed):
        self.x_linear_speed = x_linear_speed
        self.z_angular_speed = z_angular_speed
        self.r = robot
        
    def on_press(self, key):
        try:
            x_linear, z_angular = 0, 0
            if key.char in ['w', 'a', 's', 'd', 'r', 't', 'f', 'g']:
                if key.char == 'w': 
                    x_linear = self.x_linear_speed
                elif key.char == 's':
                    x_linear = -self.x_linear_speed
                elif key.char == 'a':
                    z_angular = self.z_angular_speed
                elif key.char == 'd':
                    z_angular = -self.z_angular_speed
                elif key.char == 'r' and self.x_linear_speed < 1:
                    self.x_linear_speed += 0.1
                elif key.char == 't' and self.x_linear_speed > 0:
                    self.x_linear_speed -= 0.1
                elif key.char == 'f' and self.z_angular_speed < 1:
                    self.z_angular_speed += 0.1
                elif key.char == 'g' and self.z_angular_speed > 0:
                    self.z_angular_speed -= 0.1
                
                self.r.drive.move(x_linear, z_angular)
                
        except AttributeError:
            if key == keyboard.Key.space:
                self.r.drive.move(0, 0)

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.r.stop()
            return False  # Stop listener

if __name__ == "__main__":

    hardware_spec = {
        'drive': robot.TwoWheel(name='r2wheel'),
        'button1': robot.Button('r2button1'),
        'sobe': Bool('sobe'),
        'neopixel': robot.NeoPixel(10, name = 'r2neopix'),
    }

    r = MiniRobot(hardware_spec, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)

    x_linear_speed, z_angular_speed = 1, 1

    kb = KeyboardControl(r, x_linear_speed, z_angular_speed)
    # Collect events until released
    with keyboard.Listener(
            on_press=kb.on_press,
            on_release=kb.on_release) as listener:
        # listener.join()
        last_state = False
        while True:
            curr_state = r.sobe.get()
            # print(curr_state)
            if curr_state != last_state:
                if curr_state:
                    print('ssdf')
                    x_linear_speed = 1
                    z_angular_speed = 1
                    r.neopixel.fill_with((255, 0, 0))
                    r.neopixel.write()
                else:
                    x_linear_speed = 0.4
                    z_angular_speed = 0.4
                    r.neopixel.fill_with((0, 255, 0))
                    r.neopixel.write()
            if r.button1.get():
                r.drive.move(0, 0)
                break
            last_state = curr_state
            