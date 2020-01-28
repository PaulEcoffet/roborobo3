import os
import subprocess
import sys
import time
import itertools
from typing import Sequence
import platform


def product_dict(d):
    keys = d.keys()
    vals = d.values()
    for instance in itertools.product(*vals):
        yield dict(zip(keys, instance))


def count_running(runs: Sequence[subprocess.Popen]):
    return sum(int(run.poll() is None) for run in runs)


gridconf = {
    '_generation': [203],
    '_sigma': [0.1],
    '_percentuni': [0.001],
    'fakeRobots': [True],
    'tau': [50000],
    'evaluationTime': [100000],
    'totalInvAsInput': [True],
    'gInitialNumberOfRobots': [750, 1000],
    ('wander', 'putOutOfGame', 'randomObjectPositions'): [(True, False, True), (False, True, False)]
    }

expandedgridconf = list(product_dict(gridconf))

print(len(expandedgridconf))
if len(sys.argv) == 1:
    sys.exit(0)
iconf = int(sys.argv[1]) - 1

try:
    curtime=time.localtime(int(sys.argv[2]))
except IndexError:
    curtime=time.localtime(time.time())

try:
    conffile=sys.argv[3]
except IndexError:
    conffile="config/negociate.properties"

true_type_conf = expandedgridconf[iconf]
conf = {key: val for key, val in true_type_conf.items()}


groupname = f"negociate21-bigarenawander/{time.strftime('%Y-%m-%d-%H%M', curtime)}/"
logdir = f"/home/ecoffet/robocoop/logs/{groupname}"
pythonexec = '/home/ecoffet/.virtualenvs/robocoop/bin/python'
nb_rep = 24
batch = "-b"

if platform.node().startswith('pecoffet'):
    logdir = f"/home/pecoffet/Documents/work/roborobo3/roborobo3/logs/"
    pythonexec="python"
    nb_rep = 1
    batch = ""

os.makedirs(logdir, exist_ok=True)

curruns = []
files = []
try:
    for i in range(nb_rep):
        # Lets expand the gridconf and add roborobo specific items
        robargs = []
        for key, val in conf.items():
            if isinstance(key, tuple):
                curargs = zip(['+' + str(curkey) for curkey in key], [str(v) for v in val])
                curargs = itertools.chain.from_iterable(curargs)
                robargs += curargs
            elif not key.startswith('_'):  # roborobo specific items do not start with _
                robargs += ['+'+key, str(val)]

        name = '_'.join([arg[:7] if arg.startswith('+') else arg for arg in robargs])
        curpath = logdir + '/' + name + f'/run_{i:02}/'
        os.makedirs(curpath, exist_ok=True)
        outf = open(curpath + 'out.txt', 'w')
        errf = open(curpath + 'err.txt', 'w')
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
        file_.close()

print("Over")
