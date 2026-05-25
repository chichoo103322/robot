"""Member C — Communication System.

Public API:
    SocketClient      — low-level TCP/WebSocket connection
    CommandSender     — reliable queued command sending
    HeartbeatManager  — connection health monitoring
    APIService        — high-level unified communication API (implements ICommunication)
"""

from .socket_client import SocketClient
from .command_sender import CommandSender
from .heartbeat_manager import HeartbeatManager
from .api_service import APIService
