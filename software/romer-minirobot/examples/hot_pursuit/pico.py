from romer_minirobot.urtps import uRTPS
from romer_minirobot.modules.pico import TwoWheel, Button, NeoPixel
from machine import Pin

SSID = "mechalab_intra"
PASSWORD = "mechastudent"

# SSID = "Printer_AP"
# PASSWORD = "K0van1231"

# Multicast group details
MULTICAST_GROUP = '224.0.0.252'# '224.1.1.1'
MULTICAST_TOPIC_PORT = 5007

if __name__ == "__main__":
    
    urtps = uRTPS(SSID, PASSWORD, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)
    
    urtps.add_subscribing_topics(TwoWheel(name = 'r2wheel'))
    urtps.add_publishing_topics(Button(12, Pin.PULL_UP, True, 0.1, 'r2button1'))
    urtps.add_publishing_topics(NeoPixel(28, 10, name='r2neopix'))
    
    urtps.start()