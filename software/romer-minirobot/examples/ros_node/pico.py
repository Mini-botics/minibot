from romer_minirobot.urtps import uRTPS
from romer_minirobot.modules import pico
from time import sleep


SSID = "mechalab_intra"
PASSWORD = "mechastudent"

# SSID = "Printer_AP"
# PASSWORD = "K0van1231"

# Multicast group details
MULTICAST_GROUP = '224.0.0.253'# '224.1.1.1'
MULTICAST_TOPIC_PORT = 5007

if __name__ == "__main__":
    neopix = pico.NeoPixel(28, 18)
    neopix.fillwith([255] * (18 *3))
    sleep(1)
    neopix.fillwith([0] * (18 *3))
    urtps = uRTPS(SSID, PASSWORD, MULTICAST_GROUP, MULTICAST_TOPIC_PORT)
    
    urtps.add_subscribing_topics(pico.TwoWheel())
    urtps.add_publishing_topics(pico.Button(12, 'pull_up', True, 0.1, 'isButtonPressed'))
    urtps.add_subscribing_topics(neopix)
    
    urtps.start()