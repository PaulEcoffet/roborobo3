#!/usr/env python3

import socket
from json_tricks import dump, dumps, load, loads, strip_comments
import numpy as np
import matplotlib.pyplot as plt
import subprocess
import sys
import argparse
from os.path import join
import cma

from pyevo.pyevo import getES


def recv_msg(sock, encoding='utf8'):
    """Read the whole message sent by the client through `sock`."""
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 8)
    if not raw_msglen:
        return None
    # get the message len, transform to int and get right bit order with ntohl
    msglen = socket.ntohl(int(raw_msglen.decode('utf8'), 16))
    # Read the message data
    return recvall(sock, msglen).decode(encoding)


def recvall(sock, n):
    """Read until exactly `n` bytes are read through the socket `sock`."""
    # Helper function to recv n bytes or return None if EOF is hit
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data


def send_msg(sock, msg, encoding='utf8'):
    """Send the message `msg` prefixed by its byte length through `sock`."""
    msg_byte = msg.encode(encoding)
    # write the header with the internet bit order (htonl) and encode it
    header = '{:8X}'.format(socket.htonl(len(msg_byte))).encode(encoding)
    sock.sendall(header + msg_byte)


def connect_to_open_port(serv, desired_port, ip='127.0.0.1'):
    """Find an open port to open a server and return the port chosen."""
    port = desired_port
    look_for_port = True
    while look_for_port:
        try:
            serv.bind((ip, port))
        except OSError:
            port += 1
        else:
            look_for_port = False
    return port


def main():
    # catch the output dir to put the evolution logs in it.
    ap = argparse.ArgumentParser(prog='cmaesroborobo.py')
    ap.add_argument('-o', '--output', type=str, default='logs/')
    ap.add_argument('-e', '--evolution', choices=['cmaes', 'fitprop'],
                    required=True)
    ap.add_argument('-s', '--server-only', action='store_true')
    argout, forwarded = ap.parse_known_args()
    outdir = argout.output
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as serv_s:
        desired_port = 1703
        port = connect_to_open_port(serv_s, desired_port)
        print("Open on port {}".format(port))
        serv_s.listen(1)
        # Forward the args received by pyevoroborobo to roborobo and add the port information
        if not argout.server_only:
            subprocess.Popen(['./roborobo', '-r', '127.0.0.1:{}'.format(port)] +
                             ['-o', outdir] + forwarded)

        conn, cliend_data = serv_s.accept()  # connect to roborobo
        # Wait for roborobo to give information about the simulation
        evo_info = loads(recv_msg(conn))

        es = getES(argout.evolution, evo_info['nb_weights'] * [0], 0.01,
                   evo_info['popsize'], [-1, 1], 60000, join(outdir, ''))
        sign = 1
        if argout.evolution == 'cmaes':
            sign = -1

        while not es.stop():
            solutions = [sol.tolist() for sol in es.ask()]
            send_msg(conn, dumps(solutions, primitives=True))
            ####################################
            # Roborobo simulation is done here #
            ####################################
            fit_jsonstr = recv_msg(conn)
            fitnesses = loads(fit_jsonstr)
            es.tell(solutions, np.array([sign*fit for fit in fitnesses]))
            es.disp()
            es.logger.add()
        # Close connection with roborobo (will trigger roborobo shutdown)
        conn.shutdown(socket.SHUT_RDWR)
        conn.close()
        with open(join(outdir, 'genome.txt'), 'w') as f:
            dump(es.result, f, primitives=True)
        if argout.evolution == "cmaes":
            es.logger.plot()
            cma.s.figshow()


main()
