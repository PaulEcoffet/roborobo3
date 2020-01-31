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
    '_sigma' : [0.01],
    '_percentuni' : [0.1],
    'b': [5],
    'nTolerance': [0.5, 1, 2, 3, 5, 1e25]
    'nOpti': [2, 3, 4, 20],
    'meanA': [5, 1],
    'gNbOfPhysicalObjects': [10, 20, 40, 80, 100],
    'maxPlayer': [2, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
}

expandedgridconf = list(product_dict(gridconf))


groupname = f"lionscross24-mean5paperready/{time.strftime('%Y-%m-%d-%H%M', curtime)}/"
logdir = f"/home/ecoffet/robocoop/logs/{groupname}"
pythonexec = '/home/ecoffet/.virtualenvs/robocoop/bin/python'
batch = "-b"
cluster = not platform.node().startswith('pecoffet')
nb_runs = 24
if not cluster:
    logdir = f"/home/pecoffet/Documents/work/roborobo3/roborobo3/logs/"
    pythonexec = "python"
    gridconf['_rep'] = list(range(1))
    batch = ""
    nb_runs = 1


print(int(math.ceil(len(expandedgridconfnb_runs) / nb_runs))
if len(sys.argv) == 1:
    sys.exit(0)
    
    
iconf = (int(sys.argv[1]) - 1) // nb_runs

try:
    curtime=time.localtime(int(sys.argv[2]))
except IndexError:
    curtime=time.localtime(time.time())

try:
    conffile=sys.argv[3]
except IndexError:
    conffile="config/lions_megabig.properties"


os.makedirs(logdir, exist_ok=True)

curruns = []
files = []

try:
    for i in range(nb_runs):
        # Lets expand the gridconf and add roborobo specific items
        conf = {key: val for key, val in expandedgridconf[iconf * nb_runs + i].items()}
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
