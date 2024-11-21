class Logger:
    """
    A simple logging utility class.

    Args:
        name (str): The name of the logger.
        loglevel (str): The log level to set for the logger. Valid values are 'DEBUG', 'INFO', 'WARNING', and 'ERROR'.

    Raises:
        ValueError: If an invalid log level is provided.

    Attributes:
        name (str): The name of the logger.
        loglevel (int): The log level of the logger. Can be one of Logger._DEBUG, Logger._INFO, Logger._WARNING, or Logger._ERROR.

    """

    _DEBUG = 0
    _INFO = 1
    _WARNING = 2
    _ERROR = 3

    def __init__(self, name, loglevel) -> None:
        self.name = name
        if loglevel not in ['DEBUG', 'INFO', 'WARNING', 'ERROR']:
            raise ValueError('Invalid log level')

        if loglevel == 'DEBUG':
            self.loglevel = Logger._DEBUG
        elif loglevel == 'INFO':
            self.loglevel = Logger._INFO
        elif loglevel == 'WARNING':
            self.loglevel = Logger._WARNING
        elif loglevel == 'ERROR':
            self.loglevel = Logger._ERROR

    def debug(self, msg):
        """
        Log a debug message.

        Args:
            msg (str): The message to log.

        """
        if self.loglevel <= Logger._DEBUG:
            print(f'[{self.name}] {msg}')
    
    def info(self, msg):
        """
        Log an info message.

        Args:
            msg (str): The message to log.

        """
        if self.loglevel <= Logger._INFO:
            print(f'[{self.name}] {msg}')
    
    def error(self, msg):
        """
        Log an error message.

        Args:
            msg (str): The message to log.

        """
        print(f'[{self.name}] {msg}')

    def warning(self, msg):
        """
        Log a warning message.

        Args:
            msg (str): The message to log.

        """
        if self.loglevel <= Logger._WARNING:
            print(f'[{self.name}] {msg}')