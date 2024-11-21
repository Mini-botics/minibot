import socket
import struct
import asyncio
from errno import EAGAIN
from ..utils import Logger
from .node import Node


class BaseRTPS:
    """
    The base class for uRTPS (Micro Real-Time Publish-Subscribe) communication.

    This class provides the basic functionality for uRTPS communication, including
    initializing the uRTPS base class, setting the publishing and subscribing topics,
    and managing the multicast group and port.

    Attributes:
        multicast_group (str): The multicast group address.
            Specifies the IP address of the multicast group to join.
        multicast_port (int): The multicast port number.
            Specifies the port number to use for multicast communication.
        debug (str): The debug level.
            Specifies the level of debug information to be printed.
            Valid values are 'DEBUG', 'INFO', 'WARNING', 'ERROR', and 'CRITICAL'.
        logger (Logger): The logger instance for uRTPS.
        sock (socket): The socket for uRTPS communication.
        ip_address (str): The IP address of the local machine.
        publishing_topics (Dict): A dictionary of topics to publish to.
        subscribing_topics (Dict): A dictionary of topics to subscribe to.

    Methods:
        __init__(multicast_group, multicast_port, debug='DEBUG'):
            Initialize the uRTPS base class.
        set_topics(publishing_topics, subscribing_topics):
            Set the publishing and subscribing topics for the uRTPS interface.
    """

    def __init__(self, multicast_group: str, multicast_port: int, debug='DEBUG') -> None:
        """
        Initialize the uRTPS base class.

        Args:
            multicast_group (str): The multicast group address.
                Specifies the IP address of the multicast group to join.
            multicast_port (int): The multicast port number.
                Specifies the port number to use for multicast communication.
            debug (str, optional): The debug level. Defaults to 'DEBUG'.
                Specifies the level of debug information to be printed.
                Valid values are 'DEBUG', 'INFO', 'WARNING', 'ERROR', and 'CRITICAL'.

        Returns:
            None

        Raises:
            None

        Examples:
            >>> base = BaseRTPS('224.0.0.1', 5000, 'INFO')
        """
        self.logger = Logger("uRTPS", debug)
        self.multicast_group = multicast_group
        self.multicast_port = multicast_port
        self.sock = None
        self.ip_address = None
        self.publishing_topics = {}
        self.subscribing_topics = {}

    def set_topics(self, publishing_topics, subscribing_topics):
        """
        Set the publishing and subscribing topics for the URTPS interface.

        Args:
            publishing_topics (Dict): A dict of topics to publish to.
            subscribing_topics (Dict): A dict of topics to subscribe to.
        """
        self.publishing_topics = publishing_topics
        self.subscribing_topics = subscribing_topics
    
    def add_topics(self, topics: Node|list|tuple):
        """
        Add topics to the publishing_topics or subscribing_topics dictionary.

        Args:
            topics (Node|list|tuple): The topics to be added. It can be a single Node object or a list/tuple of Node objects.

        Raises:
            None

        Returns:
            None
        """

        if isinstance(topics, (list, tuple)):
            for t in topics:
                if t.type == 'publishing':
                    self.publishing_topics[t.name] = t
                elif t.type == 'subscribing':
                    self.subscribing_topics[t.name] = t
        else:
            if topics.type == 'publishing':
                self.publishing_topics[topics.name] = topics
            elif topics.type == 'subscribing':
                self.subscribing_topics[topics.name] = topics
        
    
    def add_publishing_topics(self, topics: Node|list|tuple):
        """
        Adds publishing topics to the URTPS interface.

        Args:
            topic (Node|list|tuple): The topic(s) to be added. It can be a single Node object,
                                    or a list/tuple of Node objects.

        Returns:
            None
        """
        if isinstance(topics, (list, tuple)):
            for t in topics:
                self.publishing_topics[t.name] = t
        else:
            self.publishing_topics[topics.name] = topics
            
        
    def add_subscribing_topics(self, topic: Node|list|tuple):
        """
        Adds subscribing topics to the URTPS interface.

        Args:
            topics (Node|list|tuple): The topic or topics to be added for subscribing.

        Returns:
            None
        """
        if isinstance(topic, (list, tuple)):
            for t in topic:
                self.subscribing_topics[t.name] = t
        else:
            self.subscribing_topics[topic.name] = topic
    
    def _create_multicast_socket(self, multicast_group, multicast_port):
        """
        Create a multicast socket and configure it for the specified multicast group and port.

        Args:
            multicast_group (str): The IP address of the multicast group.
            multicast_port (int): The port number for the multicast communication.

        Returns:
            socket.socket: The created multicast socket.

        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', multicast_port))
        
        # Set the multicast group membership
        mreq = struct.pack("4sl", self._inet_aton(multicast_group), 0)  # 0 as INADDR_ANY
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        sock.setblocking(False)
        return sock

    async def _handle_subscribe(self):
        """
        Handles subscriptions by continuously receiving data from the socket and updating subscribing topics.

        This method runs in an infinite loop and continuously receives data from the socket. It decodes the received data
        and updates the subscribing topics based on the decoded message.

        Raises:
            OSError: If there is an error receiving data from the socket.

        """
        self.logger.debug('Started handling subscriptions.')
        
        while True:
            await asyncio.sleep(0)
            try:
                data, address = self.sock.recvfrom(1024)
                self.logger.debug(f"Received {len(data)} bytes from {address}: {data.decode()}")
                decoded = Node.decode(data) 
                if self.subscribing_topics.get(decoded[0]):
                    self.subscribing_topics[decoded[0]].set_message(decoded[-1])
            except OSError as e:
                if e.args[0] == EAGAIN:
                    continue
                else:
                    self.logger.error(f"Error receiving data: {e}")

    async def _handle_publishing_sequential(self):
        """
        Handles sequential publishing of messages to the multicast group.

        This method continuously loops over the publishing topics and sends their encoded messages
        to the specified multicast group and port. It uses a non-blocking sleep to allow other tasks
        to run in between iterations.

        Raises:
            Exception: If an error occurs during the publishing process.

        """
        try:
            while True:
                for topic in self.publishing_topics.values():
                    if not topic.get_message():
                        continue
                    self.sock.sendto(topic.encode(), (self.multicast_group, self.multicast_port))
                    self.logger.debug(f"Message sent to {self.multicast_group}:{self.multicast_port}: {topic.encode()}")
                await asyncio.sleep(0)
        except Exception as e:
            self.logger.error(f"Error: {e}")
        finally:
            self.sock.close()

    async def _update_sub_topics(self):
        """
        Asynchronously updates the subscribing topics.

        This method continuously updates the subscribing topics by calling their `tick` method.
        It runs in an infinite loop and uses `asyncio.sleep(0)` to allow other tasks to run.

        """
        while True:
            await asyncio.sleep(0)
            for topic in self.subscribing_topics.values():
                await topic.tick()

    async def _update_pub_topics(self):
        """
        Asynchronously updates the publishing topics.

        This method continuously updates the publishing topics by calling the `tick` method on each topic.
        It uses an infinite loop and `asyncio.sleep(0)` to allow other tasks to run in between iterations.

        Note: This method should be run in an event loop.

        Returns:
            None
        """
        while True:
            await asyncio.sleep(0)
            for topic in self.publishing_topics.values():
                await topic.tick()

    async def _main(self):
        """
        Main method for uRTPS functionality.

        This method starts the uRTPS process, establishes a connection to the multicast group,
        and creates tasks for handling subscriptions, publishing sequentially, and updating
        publish and subscribe topics. It waits for all tasks to complete using `asyncio.gather`.

        Returns:
            None
        """
        self.logger.debug('Starting uRTPS.')
        if not self.connect():
            return
        self.logger.debug('Connected to multicast group.')

        tasks = [
            asyncio.create_task(self._handle_subscribe()),
            asyncio.create_task(self._handle_publishing_sequential()),
            asyncio.create_task(self._update_pub_topics()),
            asyncio.create_task(self._update_sub_topics())
        ]
        await asyncio.gather(*tasks)
    
    def start(self):
        """
        Starts the execution of the URTPS interface.

        This method runs the main event loop of the URTPS interface, allowing it to send and receive messages.
        """
        asyncio.run(self._main())

    @staticmethod
    def _inet_aton(ip):
        """
        Convert an IP address string to its packed 32-bit binary format.

        Args:
            ip (str): The IP address string in the format 'x.x.x.x'.

        Returns:
            bytes: The packed binary representation of the IP address.

        """
        return bytes(map(int, ip.split('.')))