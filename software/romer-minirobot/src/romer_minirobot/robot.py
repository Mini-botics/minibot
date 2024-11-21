from .urtps import uRTPS

from .modules import robot
from .utils import Logger

class MiniRobot:
    
    def __init__(self, hardware_spec: dict, multicast_group, multicast_port, debug = 'DEBUG') -> None:
        # Initialize the logger
        self.logger = Logger("MiniRobot", debug)
        
        # Initialize the Message Passing Interface
        self.mpi = uRTPS(multicast_group, multicast_port)
        
        # Initialize the hardware
        for key, value in hardware_spec.items():
            setattr(self, key, value)
                
        self.mpi.add_topics(list(hardware_spec.values()))
        self.init()
     
    def init(self):
        # Start the Message Passing Interface
        self.mpi._start_async_main_in_thread()
        self.logger.debug('MiniRobot started Message Passing Interface.')
        self.logger.debug('MiniRobot started.')

    def stop(self):
        # Stop the Message Passing Interface
        self.mpi.stop()
        self.logger.debug('MiniRobot stopped.')
