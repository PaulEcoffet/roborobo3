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
gridconf['_confname'] = ['negociate']
gridconf['_generation'] = [100]
gridconf['_sigma'] = [0.1]
gridconf['_percentuni'] = [0.001]

gridconf['mutProbCoop'] = [0.2]
gridconf['fakeRobots'] = [False, True]
gridconf['fakeCoef'] = [1]
gridconf['mutRate'] = [0.1]
gridconf['mutCoop'] = [0.1]
gridconf['doNotKill'] = ['false']
gridconf['tau'] = [10000, 1000, 0]
gridconf['nbEvaluationsPerGeneration'] = [1, 10]
gridconf['gInitialNumberOfRobots'] = [150, 100, 50, 20]

expandedgridconf = list(product_dict(**gridconf))
print(len(expandedgridconf))
iconf = int(sys.argv[1]) - 1

true_type_conf = expandedgridconf[iconf]
conf = {key: str(val) for key, val in true_type_conf.items()}


groupname = f"negociate11/"
logdir = f"/home/ecoffet/robocoop/logs/{groupname}"
if platform.node().startswith('pecoffet'):
    logdir = f"/home/pecoffet/Document/work/roborobo3/roborobo3/logs/"

os.makedirs(logdir, exist_ok=True)
pythonexec = '/home/ecoffet/.virtualenvs/robocoop/bin/python'


nb_rep = 20


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

for i in range(nb_rep):
    name = '+'.join([str(key) + '_' + str(val)
                     for (key, val) in conf.items() if not key.startswith('_')])
    args = [pythonexec, 'pyevoroborobo.py',
            '--no-movie',
            '-e', 'fitprop',
            '--percentuni', conf['_percentuni'],
            '-g', conf['_generation'],
            '-s', conf['_sigma'],
            '-l', f'config/{conf["_confname"]}.properties',
            '-p', '1',
            '-o', logdir + '/' + name + f'/run_{i:02}/',
            #'-b',
            '--'
            ]
    # Lets add the option in + notation
    for key, val in conf.items():
        if not key.startswith('_'):
            args += ['+'+key, str(val)]
    print(' '.join(args))
    run = subprocess.Popen(args)
    curruns.append(run)

    while count_running(curruns) >= 24:
        time.sleep(5)

print("Over")
