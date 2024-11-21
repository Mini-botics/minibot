from ..utils import is_running_on_pico
from .baseurtps import BaseRTPS

CONN_TIMEOUT = 10 # Connection timeout in seconds

if is_running_on_pico():
    from .urtpspi import uRTPSPi as uRTPS
else:
    from .urtps import uRTPS

from .node import Node, EventPubNode, EventSubNode, BlockingNode, BaseNode