from romer_minirobot.robot import MiniRobot
from romer_minirobot.modules import robot
from pynput import keyboard
import time
        
# Multicast group details
MULTICAST_GROUP = '224.0.0.252'
MULTICAST_TOPIC_PORT = 5007
G = (0,255,0)
R = (255,0,0)

hardware_spec = {
    'drive': robot.Holonomic(),
    'bumper': robot.Button('bumper'),
    'neopixel' : robot.NeoPixel(18, 0.5, name = 'headlight'),
}

r = MiniRobot(hardware_spec, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)
r.neopixel.fill_with(G)
r.neopixel.write()

x_linear_speed, y_linear_speed, z_angular_speed = 1, 1, 1

def on_press(key):
    global x_linear_speed, z_angular_speed
    try:
        x_linear, y_linear, z_angular = 0, 0, 0
        if key.char in ['w', 'a', 's', 'd', 'q', 'e']:
            if key.char == 'w': 
                x_linear = x_linear_speed
            elif key.char == 's':
                x_linear = -x_linear_speed
            elif key.char == 'a':
                y_linear = y_linear_speed
            elif key.char == 'd':
                y_linear = -y_linear_speed
            elif key.char == 'q':
                z_angular = z_angular_speed
            elif key.char == 'e':
                z_angular = -z_angular_speed
            
            r.drive.move(x_linear, y_linear, z_angular)
            # print(f'{key.char.upper()} key pressed')
    except AttributeError:
        if key == keyboard.Key.space:
            r.drive.move(0, 0, 0)
            # print('Space key pressed')

def on_release(key):
    if key == keyboard.Key.esc:
        r.stop()
        return False  # Stop listener

if __name__ == "__main__":

    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        # listener.join()
        while True:
            if r.bumper.get():
                r.neopixel.fill_with(R)
            else:
                r.neopixel.fill_with(G)
            r.neopixel.write()
            time.sleep(0.3)

        


