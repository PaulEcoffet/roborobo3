import os
import subprocess
import sys
import time
import itertools
from typing import Sequence
import platform


def product_dict(**kwargs):
    keys = kwargs.keys()
    vals = kwargs.values()
    for instance in itertools.product(*vals):
        yield dict(zip(keys, instance))


def count_running(runs: Sequence[subprocess.Popen]):
    return sum(int(run.poll() is None) for run in runs)


gridconf = {}
gridconf['_generation'] = [203]
gridconf['_sigma'] = [0.1]
gridconf['_percentuni'] = [0.001]

gridconf['logEveryXGen'] = [50]
gridconf['nbEvaluationsPerGeneration'] = [1]
gridconf['fakeRobots'] = [True]
gridconf['mutCoop'] = [0.1]
gridconf['fakeCoef'] = [1]
gridconf['mutRate'] = [0.1]
gridconf['mutProbCoop'] = [0.01]
gridconf['doNotKill'] = ['false']
gridconf['tau'] = [50000]
gridconf['evaluationTime'] = [100000]
gridconf['totalInvAsInput'] = [True]
gridconf['gInitialNumberOfRobots'] = [750, 1000]
gridconf['wander+putOutOfGame+randomObjectPositions'] = [(True, False, True), (False, True, False)]

expandedgridconf = list(product_dict(**gridconf))

try:
    curtime=time.localtime(int(sys.argv[2]))
except Exception:
    curtime=time.localtime(time.time())

try:
    conffile=sys.argv[3]
except:
    conffile="config/negociate.properties"

print(len(expandedgridconf))
if len(sys.argv) == 1:
    sys.exit(0)
iconf = int(sys.argv[1]) - 1

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




# First we train <3
# for curconf in "${donotkills[@]}"
# do
#     echo "starting training $curconf"
#     name="donotkill_$curconf"
#     for i in `seq -f "%02.f" $nb_rep`
#     do
#         $python pyevoroborobo.py --no-movie -e fitprop --percentuni $percentuni -g 50 -s $sigma -l "config/negociate.properties" -o $logdir/$name/run_$i/train/ -p 1 -b -- +oppDecay 0 +evaluationTime 5000 +gInitialNumberOfRobots $nbrob +fakeRobots $fakeRobots +fakeCoef $fakecoef +mutRate $mutRate +mutCoop 0 +train true +nbEvaluationsPerGeneration 5 +logEveryXGen 10 +mutRate 0.02 +mutProb 0.000001 &
#     done
# done

# wait  # everything must be over here !

curruns = []
files = []
try:
    for i in range(nb_rep):
        # Lets add the option in + notation
        robargs = []
        for key, val in conf.items():
            if not key.startswith('_'):
                if '+' in key:
                    splitparams = key.split('+')
                    curargs = [['+'+curkey, str(val[i])] for i, curkey in enumerate(splitparams)]
                    curargs = itertools.chain.from_iterable(curargs)
                    robargs += curargs
                else:
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
