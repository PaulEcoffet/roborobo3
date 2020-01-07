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
gridconf['fakeRobots'] = [False, True]
gridconf['mutCoop'] = [0.1]
gridconf['fakeCoef'] = [1]
gridconf['mutRate'] = [0.1]
gridconf['mutProbCoop'] = [0.01]
gridconf['doNotKill'] = ['false']
gridconf['tau'] = [0, 100, 500, 1000, 5000, 10000]
gridconf['evaluationTime'] = [100000]
gridconf['totalInvAsInput'] = [True]
gridconf['gInitialNumberOfRobots'] = [10, 20, 50, 100, 200, 300, 500, 750, 1000]

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
conf = {key: str(val) for key, val in true_type_conf.items()}


groupname = f"negociate17-3tau/{time.strftime('%Y-%m-%d-%H%M', curtime)}/"
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
        name = '+'.join([str(key) + '_' + str(val)
                         for (key, val) in conf.items() if not key.startswith('_')])
        curpath = logdir + '/' + name + f'/run_{i:02}/'
        os.makedirs(curpath, exist_ok=True)
        outf = open(curpath + 'out.txt', 'w')
        errf = open(curpath + 'err.txt', 'w')
        files += [outf, errf]
        args = [pythonexec, 'pyevoroborobo.py',
                '--no-movie',
                '-e', 'fitprop',
                '--percentuni', conf['_percentuni'],
                '-g', conf['_generation'],
                '-s', conf['_sigma'],
                '-l', conffile,
                '-p', '1',
                '-o', curpath,
                batch,
                '--'
                ]
        # Lets add the option in + notation
        for key, val in conf.items():
            if not key.startswith('_'):
                args += ['+'+key, str(val)]
        print(' '.join(args))
        run = subprocess.Popen(args, stdout=outf, stderr=errf)
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
