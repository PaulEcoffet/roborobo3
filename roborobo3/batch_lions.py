import os
import subprocess
import sys
import time
import itertools
from typing import Sequence
import platform
import math

def product_dict(d):
    keys = d.keys()
    vals = d.values()
    for instance in itertools.product(*vals):
        yield dict(zip(keys, instance))


def count_running(runs: Sequence[subprocess.Popen]):
    return sum(int(run.poll() is None) for run in runs)


gridconf = {
    '_rep': list(range(24)),
    '_generation': [1500],
    '_sigma' : [0.01],  # ignored, actually controlled in mutProb
    '_percentuni': [0.1],
    'nOpti': [2, 3, 4],
    'costAsInput': [True],
    'gInitialNumberOfRobots': [100],
    'gNbOfPhysicalObjects': list(range(10, 101, 10)),
    'maxPlayer': [2, 5] + list(range(10, 101, 10)),
    'cost': [0],
    'nTolerance': [0.1, 0.5, 1.5, 2]
}

def implies(p, q):
    return ((not p) or q)

def allow(conf):
    tests = [
            lambda x: implies(x['nTolerance'] > 1e20, x['nOpti'] == 2),
            lambda x: implies(x['gNbOfPhysicalObjects'] not in [20, 40, 80], x['maxPlayer'] == 100),
            ]
    return all(test(conf) for test in tests)


expandedgridconf = list(conf for conf in product_dict(gridconf) if allow(conf))


try:
    curtime=time.localtime(int(sys.argv[2]))
except IndexError:
    curtime=time.localtime(time.time())



cluster = not platform.node().startswith('pecoffet')
nb_runs = 72
groupname = f"lionscross24-mean5paperready/{time.strftime('%Y-%m-%d-%H%M', curtime)}/"
logdir = f"/home/ecoffet/robocoop/logs/{groupname}"
pythonexec = '/home/ecoffet/.virtualenvs/robocoop/bin/python'
batch = "-b"
if not cluster:
    logdir = f"/home/pecoffet/Documents/work/roborobo3/roborobo3/logs/"
    pythonexec = "python"
    gridconf['_rep'] = list(range(1))
    batch = ""
    nb_runs = 1

from math import ceil
print(int(ceil((len(expandedgridconf) / nb_runs))))
if len(sys.argv) == 1:
    sys.exit(0)

os.makedirs(logdir, exist_ok=True)



iconf = (int(sys.argv[1]) - 1)

try:
    conffile=sys.argv[3]
except IndexError:
    conffile="config/lion_megabig.properties"



curruns = []
files = []

try:
    for i in range(nb_runs):
        # Lets expand the gridconf and add roborobo specific items
        try:
            conf = {key: val for key, val in expandedgridconf[iconf * nb_runs + i].items()}
        except IndexError:  # No more run to do
            break
        robargs = []
        for key, val in conf.items():
            if isinstance(key, tuple):
                curargs = zip(['+' + str(curkey) for curkey in key], [str(v) for v in val])
                curargs = itertools.chain.from_iterable(curargs)
                robargs += curargs
            elif not key.startswith('_'):  # roborobo specific items do not start with _
                robargs += ['+' + key, str(val)]

        name = '_'.join([arg[:7] if arg.startswith('+') else arg for arg in robargs])
        curpath = logdir + '/' + name + f'/run_{conf["_rep"]:02}/'
        os.makedirs(curpath, exist_ok=True)
        if cluster:
            outf = open(curpath + 'out.txt', 'w')
            errf = open(curpath + 'err.txt', 'w')
        else:
            outf = None
            errf = None
        files += [outf, errf]

        args = [pythonexec, 'pyevoroborobo.py',
                '--no-movie',
                '-e', 'fitprop',
                '--percentuni', str(conf['_percentuni']),
                '-g', str(conf['_generation']),
                '-s', str(conf['_sigma']),
                '-l', conffile,
                '-p', '1',
                '-o', curpath,
                batch,
                '--'
                ]
        print(' '.join(args + robargs))
        run = subprocess.Popen(args + robargs, stdout=outf, stderr=errf)
        curruns.append(run)

        while count_running(curruns) >= 24:
            time.sleep(10)
    # wait until everyone finished
    while not count_running(curruns) == 0:
        time.sleep(10)
finally:
    for run in curruns:
        if run.poll() is None:
            run.kill()
    for file_ in files:
        if file_ is not None:
            file_.close()
print("Over")
