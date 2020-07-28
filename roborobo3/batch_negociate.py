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


project='negociatesmall'
runid=1
runname='bench'
subname='all'
gridconf = {
    'gInitialNumberOfRobots': [100, 10, 20, 50, 70, 90],
    'tau': [0, 100000, 50000, 10000, 1000],
    '_rep': list(range(24//2)),
    '_generation': [203],
    '_sigma': [0.1],
    '_percentuni': [0.01],
    '_algo': ['fitprop'],
#    '_guess': ['trainedgenomes/negociate_nav_{}.txt']
    'fakeRobots': [False],
    'mutCoop': [0.1],
    'mutProb': [0.01],
    'mutRate': [0.01],
    'mutProbCoop': [0.1],
    'mutProbNegociate': [0.01],
    'evaluationTime': [100000],
    'totalInvAsInput': [True, False],
    ('wander', 'putOutOfGame', 'randomObjectPositions'): [(True, False, True)],
    'train': [0]
    }


expandedgridconf = list(product_dict(gridconf))


try:
    curtime=time.localtime(int(sys.argv[2]))
except IndexError:
    curtime=time.localtime(time.time())



cluster = 'cluster' in platform.node()
nb_runs = 24
groupname = f"{project}{runid}-{runname}-{subname}/{time.strftime('%Y-%m-%d-%H%M', curtime)}/"
logdir = f"/home/ecoffet/robocoop/logs/{groupname}"
pythonexec = '/home/ecoffet/.virtualenvs/robocoop/bin/python'
batch = "-b"
if not cluster:
    logdir = f"/home/pecoffet/Documents/work/roborobo3/roborobo3/logs/"
    pythonexec = "python"
    gridconf['_rep'] = list(range(1))
    batch = ""
    nb_runs = 1

print(int(math.ceil(len(expandedgridconf) / nb_runs)))


if len(sys.argv) == 1:
    sys.exit(0)

os.makedirs(logdir, exist_ok=True)



iconf = (int(sys.argv[1]) - 1)

try:
    conffile=sys.argv[3]
except IndexError:
    conffile="config/negociate_small.properties"



curruns = []
files = []

try:
    for i in range(nb_runs):
        # wait until there is room to start runs
        while count_running(curruns) >= 24:
            time.sleep(10)

        # Load current run
        try:
            conf = {key: val for key, val in expandedgridconf[iconf * nb_runs + i].items()}
        except IndexError: # If there is no more run to do
            break
        except Exception as e:
            print("not the expected Exception")
            print(type(e), str(e))
            break

        # Else let's start runs
        # Lets expand the gridconf and add roborobo specific items
        robargs = []
        for key, val in conf.items():
            if isinstance(key, tuple):
                curargs = zip(['+' + str(curkey) for curkey in key], [str(v) for v in val])
                curargs = itertools.chain.from_iterable(curargs)
                robargs += curargs
            elif not key.startswith('_'):  # roborobo specific items do not start with _
                robargs += ['+' + key, str(val)]

        name = '_'.join([arg[:7] if arg.startswith('+') else arg for arg in robargs])
        if '_algo' in conf:
            name += '_' + conf['_algo']
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
                '-e', str(conf['_algo']),
                '--percentuni', str(conf['_percentuni']),
                '-g', str(conf['_generation']),
                '-s', str(conf['_sigma']),
                '-l', conffile,
                '-p', '1',
                '-o', curpath,
                batch,
                ]
        if 'guess' in conf:
            args += ['--guess', conf['guess'].format(conf["gInitialNumberOfRobots"])]
        args += ['--']
        print(' '.join(args + robargs))
        run = subprocess.Popen(args + robargs, stdout=outf, stderr=errf)
        curruns.append(run)

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
