#!/usr/env python3

import socket
from json_tricks import dump, dumps, load, loads, strip_comments
import numpy as np


import subprocess
import sys
import argparse
from os.path import join
import tqdm
from pathlib import Path

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
    header = '{:8X}'.format(len(msg_byte)).encode(encoding)
    sock.sendall(header + msg_byte)


def connect_to_open_port(serv, desired_port, ip=None):
    """Find an open port to open a server and return the port chosen."""
    if ip is None:
        ip = '127.0.0.1'
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
    ap.add_argument('-e', '--evolution', choices=['cmaes', 'fitprop', 'mulambda', 'oneone', 'noneone'],
                    required=True)
    ap.add_argument('-m', '--mu', type=int, default=1)
    ap.add_argument('--guess', type=argparse.FileType())
    ap.add_argument('-s', '--sigma', type=float, default=0.01)
    ap.add_argument('--server-only', action='store_true')
    ap.add_argument('-p', '--parallel-rep', type=int, default=1)
    ap.add_argument('-g', '--generations', type=int, default=30000)
    ap.add_argument('--movie', dest='movie', action='store_true')
    ap.add_argument('--no-movie', dest='movie', action='store_false')
    ap.add_argument('--percentuni', type=float, default=0.1)
    ap.set_defaults(movie=False)
    argout, forwarded = ap.parse_known_args()
    outdir = Path(argout.output)
    sys.stdout.write("\x1b]2;{}\x07".format(outdir.name))  # Change the terminal title
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as serv_s:
        desired_port = 1703
        port = connect_to_open_port(serv_s, desired_port)
        print("Open on port {}".format(port), flush=True)
        serv_s.listen(10)
        # Forward the args received by pyevoroborobo to roborobo and add the port information
        print("Forwarded:", forwarded, flush=True)
        print(' '.join(['./roborobo', '-r', '127.0.0.1:{}'.format(port)] +
                       ['-o', str(outdir / 'rep{:02}'.format(0))] + forwarded + ['+takeVideo', 'false']))
        if not argout.server_only:
            movie = str(argout.movie)  # output movie only for the first one
            for i in range(argout.parallel_rep):
                subprocess.Popen(['./roborobo', '-r', '127.0.0.1:{}'.format(port)] +
                                 ['-o', str(outdir / 'rep{:02}'.format(i))] + forwarded + ['+takeVideo', movie])
                movie = 'false'
        conns = []
        client_datas = []
        for i in range(argout.parallel_rep):
            print("try to connect", flush=True)
            tmpconn, tmpclient_data = serv_s.accept()  # connect to roborobo
            conns.append(tmpconn)
            client_datas.append(tmpclient_data)
            print('*************************connected to {}************************'.format(i), flush=True)
            # Wait for roborobo to give information about the simulation
            evo_info = loads(recv_msg(conns[i]))
        if 'min_bounds' in evo_info:
            bounds = [evo_info['min_bounds'], evo_info['max_bounds']]
        else:
            # Evolution bounds
            minb = -10
            maxb = 10
            bounds = np.array([[0] + [minb] * (evo_info['nb_weights'] - 1),
                               [1] + [maxb] * (evo_info['nb_weights'] - 1)])
        if 'min_guess' in evo_info:
            init_min = evo_info['min_guess']
            init_max = evo_info['max_guess']
        else:
            # Init bounds
            init_min = np.array([0] + [-1] * (evo_info['nb_weights'] - 1))
            init_max = np.array([0.01] + [1] * (evo_info['nb_weights'] -1))
        normalmut = None
        if 'std' in evo_info:
            normalmut = evo_info['std']
        else:
            normalmut = np.array([0.1] * evo_info['nb_weights'])
        if 'mutprob' in evo_info:
            mutprob=np.asarray(evo_info['mutprob'])
        else:
            mutprob=1/(evo_info['nb_weights'] * evo_info['popsize'])
        if argout.guess:
            guess = np.asarray(load(argout.guess))
            assert((len(guess.shape) == 1 and guess.shape[0] == evo_info['nb_weights'])
                   or (len(guess.shape) == 2 and guess.shape[1] == evo_info['nb_weights']))
        else:
            guess = lambda: np.random.uniform(init_min, init_max)
        es = getES(argout.evolution,
                   guess,
                   argout.sigma,
                   evo_info['popsize'], bounds, argout.generations, join(str(outdir), ''), mu=argout.mu,
                   normalmut=normalmut, percentuni=argout.percentuni, mutprob=mutprob)
        sign = 1
        if argout.evolution == 'cmaes':
            sign = -1

        end = False
        prog = tqdm.tqdm(leave=False, position=0, total=argout.generations)
        while not es.stop() and not end:
            solutions = [sol.tolist() for sol in es.ask()]
            for i in range(argout.parallel_rep):
                send_msg(conns[i], dumps(solutions, primitives=True))
            ########################################
            # Roborobo simulation(s) are done here #
            ########################################
            fitnesses = []
            for i in range(argout.parallel_rep):
                back_jsonstr = recv_msg(conns[i])
                if back_jsonstr is None:
                    end = True
                    break
                back_json = loads(back_jsonstr)
                assert(np.allclose(back_json['ind'], np.array(solutions)))
                fitnesses.append(back_json['fitness'])
            if not end:
                fitnesses = np.asarray(fitnesses).mean(axis=0)
                es.tell(solutions, np.array([sign*fit for fit in fitnesses]))
                es.disp()
                sys.stdout.flush()
                es.logger.add()
                prog.update(1)
        # Close connection with roborobo (will trigger roborobo shutdown)
        for i in range(argout.parallel_rep):
            conns[i].shutdown(socket.SHUT_RDWR)
            conns[i].close()
        with open(join(str(outdir), 'genome_end.txt'), 'w') as f:
            dump(solutions, f, primitives=True)
        prog.close()

main()
