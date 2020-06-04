from typing import *
from gymroborobo.network import RoboroboConnection
import numpy as np
import subprocess
import io


class Roborobo:
    """
    Roborobo bridge between python through TCP
    """

    def __init__(self):
        self.roborobo_connection = RoboroboConnection()
        self.roborobo_process = None

    def start(self, conf: Optional[Dict[str, Any]] = None):
        port = self.roborobo_connection.connect_to_open_port(1703)
        conflistout = []
        if conf:
            confdata = list(conf.items())
            for elem in confdata:
                conflistout += ['+' + elem[0], str(elem[1])]
        self.roborobo_process = subprocess.Popen(['../roborobo3/roborobo', '-r', '127.0.0.1:{}'.format(port)] +
                                                 conflistout)

    def close(self):
        self.roborobo_process.kill()
        self.roborobo_connection.close()

    def _get_obs(self):
        msg = self.roborobo_connection.recv_msg()
        msg_stream = io.BytesIO(msg)
        return np.load(msg_stream)

    def send_actions(self, actions: np.array):
        actions_bytestream = io.BytesIO()
        np.save(actions_bytestream, actions)
        self.roborobo_connection.send_msg(actions_bytestream.getbuffer())
        return self._get_obs()
