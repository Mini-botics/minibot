from romer_minirobot.robot import MiniRobot
from romer_minirobot.modules import robot
from time import sleep

# Multicast group details
MULTICAST_GROUP = '224.0.0.252'
MULTICAST_TOPIC_PORT = 5007

hardware_spec = {
    'bumper': robot.Button('bumper'),
    'neopixel' : robot.NeoPixel(18, 0.5),
}
G = (0,255,0)
R = (255,0,0)

r = MiniRobot(hardware_spec, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)
r.neopixel.fill_with(G)
r.neopixel.write()

while True:
    if r.bumper.get():
        r.neopixel.fill_with(R)
    else:
        r.neopixel.fill_with(G)
    r.neopixel.write()
    sleep(0.1)