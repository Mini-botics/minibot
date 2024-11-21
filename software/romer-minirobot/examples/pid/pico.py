from romer_minirobot.urtps import uRTPS
from romer_minirobot.modules.pico import TwoWheelPID, Button

SSID = "mechalab_intra"
PASSWORD = "mechastudent"

# SSID = "Printer_AP"
# PASSWORD = "K0van1231"

# Multicast group details
MULTICAST_GROUP = '224.0.0.253'# '224.1.1.1'
MULTICAST_TOPIC_PORT = 5007

if __name__ == "__main__":
    
    urtps = uRTPS(SSID, PASSWORD, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)
    
    urtps.add_subscribing_topics(TwoWheelPID())
    urtps.add_publishing_topics(Button(12, 'pull_up', True, 0.1, 'button1'))
    
    urtps.start()