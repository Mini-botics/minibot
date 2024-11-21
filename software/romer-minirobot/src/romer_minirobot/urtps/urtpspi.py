import network
import utime
from . import CONN_TIMEOUT, BaseRTPS


class uRTPSPi(BaseRTPS):
    def __init__(self, wifi_ssid, wifi_password, multicast_group='224.0.0.253', multicast_port=5007, debug='DEBUG') -> None:
        """
        uRTPSPi class represents a uRTPS (Micro Real-Time Publish-Subscribe) client for the Pico board.
        It provides functionality to connect to Wi-Fi, create a multicast socket, handle subscriptions, and publish messages.

        Args:
            wifi_ssid (str): The SSID of the Wi-Fi network to connect to.
            wifi_password (str): The password of the Wi-Fi network.
            multicast_group (str, optional): The multicast group IP address. Defaults to '224.0.0.253'.
            multicast_port (int, optional): The multicast port number. Defaults to 5007.
            debug (str, optional): The debug level. Defaults to 'DEBUG'.

        Attributes:
            logger (Logger): The logger instance for logging debug and error messages.
            wifi_ssid (str): The SSID of the Wi-Fi network.
            wifi_password (str): The password of the Wi-Fi network.
            multicast_group (str): The multicast group IP address.
            multicast_port (int): The multicast port number.
            sock (socket.socket): The multicast socket.
            ip_address (str): The IP address assigned to the Pico board.
            publishing_topics (dict): A dictionary of publishing topics.
            subscribing_topics (dict): A dictionary of subscribing topics.
        """
        super().__init__(multicast_group, multicast_port, debug)
        self.wifi_ssid = wifi_ssid
        self.wifi_password = wifi_password
        
    def _connect_wifi(self, ssid: str, password: str):
        """
        Connects to a Wi-Fi network using the provided SSID and password.

        Args:
            ssid (str): The SSID of the Wi-Fi network.
            password (str): The password for the Wi-Fi network.

        Returns:
            str: The IP address assigned to the device after successful connection.

        Raises:
            None

        """
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(ssid, password)

        self.logger.debug('Connecting to Wi-Fi...')
        
        # Wait until the connection is established with polling
        start_time = utime.ticks_ms()
        while not wlan.isconnected():
            current_time = utime.ticks_ms()
            if current_time - start_time > CONN_TIMEOUT * 1000:
                self.logger.error("Wifi Connection timed out.")
                self.logger.error('Quitting.')                
                return None

        self.ip_address = wlan.ifconfig()[0]
        
        self.logger.debug(f'Connected to Wi-Fi at: {self.ip_address}')
        
        return self.ip_address

    def connect(self):
        """
        Connects to the Wi-Fi network and creates a multicast socket.

        Returns:
            socket: The created multicast socket if successful, None otherwise.
        """
        if not self._connect_wifi(self.wifi_ssid, self.wifi_password):
            self.logger.error("Could not connect to Wi-Fi.")
            return None
        self.sock = self._create_multicast_socket(self.multicast_group, self.multicast_port)
        if not self.sock:
            self.logger.error("Could not create multicast socket.")
            return None
        return self.sock