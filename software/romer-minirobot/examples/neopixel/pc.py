from romer_minirobot.robot import MiniRobot
from romer_minirobot.modules import robot
from time import sleep

# Multicast group details
MULTICAST_GROUP = '224.0.0.252'
MULTICAST_TOPIC_PORT = 5007

hardware_spec = {
    'button1': robot.Button('button1'),
    'neopixel' : robot.NeoPixel(18, 0.5),
}

r = MiniRobot(hardware_spec, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)

x = 1
while True:
    if r.button1.get():
        color = (255*x,255*(1-x),0)
        r.neopixel.fill_with(color)
        r.neopixel.write()
        x += 0.5
        sleep(0.3)