import _thread
from . import BaseRTPS


class uRTPS(BaseRTPS):
    def __init__(self, multicast_group='224.0.0.253', multicast_port=5007, debug='DEBUG') -> None:
        """
        Initializes the URTPS (micro Real-Time Publish-Subscribe) object.

        Args:
            multicast_group (str): The multicast group IP address to use for communication. Default is '224.0.0.253'.
            multicast_port (int): The multicast port number to use for communication. Default is 5007.
            debug (str): The debug level for logging. Default is 'DEBUG'.

        Returns:
            None
        """
        super().__init__(multicast_group, multicast_port, debug)
        self._thread_running = _thread.allocate_lock()
        
    def connect(self):
        """
        Connects to the specified multicast group and port.

        This method creates a multicast socket and attempts to connect to the specified multicast group and port.
        If the connection is successful, the socket object is returned. Otherwise, an error message is logged and None is returned.

        Returns:
            socket: The socket object if the connection is successful.
            None: If the connection fails.

        """
        self.sock = self._create_multicast_socket(self.multicast_group, self.multicast_port)
        if not self.sock:
            self.logger.error("Could not create multicast socket.")
            return None
        return self.sock
    
    def stop(self):
        """
        Stops the uRTPS communication.

        This method releases the thread lock, closes the socket connection, and stops the uRTPS communication.
        After calling this method, the uRTPS communication will be completely stopped.

        Note:
        - If the uRTPS communication is already stopped, calling this method has no effect.

        """
        self._thread_running.release()
        self.logger.debug('uRTPS stopped.')
        if self.sock:
            self.sock.close()
            
    def _start_async_main_in_thread(self):
        """
        Starts the uRTPS in a new thread.

        This method is responsible for starting the uRTPS (micro Real-Time Publish-Subscribe) communication
        framework in a new thread. It logs the start of the thread and returns the thread ID.

        Returns:
            int: The thread ID of the newly started thread.

        Example:
            >>> urtps = URTPS()
            >>> urtps._start_async_main_in_thread()
            Starting uRTPS in a new thread.
            Thread ID: 12345
        """
        self.logger.debug('Starting uRTPS in a new thread.')
        idf = _thread.start_new_thread(self.start, ())
        self.logger.debug(f'Thread ID: {idf}')

    class URTPS:
        def wait_until_complete(self):
            """
            Waits until the uRTPS process is complete.

            This method starts the uRTPS process in a new thread and waits until it completes.
            It uses a thread lock to ensure synchronization between the main thread and the uRTPS thread.

            Returns:
                None

            Example:
                >>> urtps = URTPS()
                >>> urtps.wait_until_complete()
                # The uRTPS process starts in a new thread.
                # The main thread waits until the uRTPS process completes.
                # Once the process is complete, the method returns None.

            Raises:
                Any exceptions raised by the uRTPS process.

            """
            self.logger.debug('Starting uRTPS in a new thread.')

            idf = _thread.start_new_thread(self.start, ())
            self._thread_running.acquire()
            self.logger.debug(f'Thread ID: {idf}, waiting until complete.')

            while self._thread_running.locked():
                pass
        