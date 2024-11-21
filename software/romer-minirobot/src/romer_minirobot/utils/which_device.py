import sys

def is_running_on_pico():
    """
    Check if the code is running on a Pico board.

    Returns:
        bool: True if running on a Pico board, False otherwise.
    """
    if 'MicroPython' in sys.version:
        return True
    return False

def is_running_on_windows():
    """
    Check if the code is running on a Windows machine.

    Returns:
        bool: True if running on a Windows machine, False otherwise.
    """
    if 'win32' in sys.platform:
        return True
    return False