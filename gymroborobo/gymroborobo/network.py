import io
import socket
from typing import *


class RoboroboConnection:
    def __init__(self):
        self.socket: Optional[socket.socket] = None
        self.connection_data = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __del__(self):
        self.close()

    def close(self):
        self.socket.close()
        self.server.close()

    def accept_connection(self):
        self.socket, self.connection_data = self.server.accept()

    def recv_msg(self, encoding: Optional[str] = None) -> Optional[Union[bytes, str]]:
        """Read the whole message sent by the client through `sock`."""

        # Read message length and unpack it into an integer
        raw_msg_len = self._recvall(8)
        if not raw_msg_len:
            return None
        # get the message len
        msg_len = int(raw_msg_len, 16)
        # Read the message data
        res = self._recvall(msg_len)
        if encoding:
            return res.decode(encoding)
        else:
            return res

    def _recvall(self, n):
        """Read until exactly `n` bytes are read through the socket `sock`."""
        data = b''
        while len(data) < n:
            packet = self.socket.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def send_msg(self, msg, encoding=None):
        """Send the message `msg` prefixed by its byte length through `sock`."""
        if encoding:
            msg_byte = msg.encode(encoding)
        else:
            msg_byte = bytes(msg)
        header = '{:8X}'.format(len(msg_byte))
        header = bytes(header, 'utf8')
        self.socket.sendall(header + msg_byte)

    def connect_to_open_port(self, desired_port, ip=None):
        """Find an open port to open a server and return the port chosen."""
        if ip is None:
            ip = '127.0.0.1'
        port = desired_port
        look_for_port = True
        while look_for_port:
            try:
                print(ip, port)
                self.server.bind((ip, port))
            except OSError as e:
                print(e)
                port += 1
            else:
                look_for_port = False
        self.server.listen(10)
        return port
