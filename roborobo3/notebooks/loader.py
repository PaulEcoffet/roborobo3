import numpy as np
import pandas as pd
import json
from pathlib import Path
import glob
import re
import os.path
from os.path import join

def read_csvx(path, *args, **kwargs):
    try:
        return pd.read_csv(str(path) +'.gz', *args, **kwargs)
    except Exception as e:
        return pd.read_csv(str(path), *args, **kwargs)

    
def getlogs(paths, gens=None):
    if gens is None:
        autogen = True
    else:
        autogen = False
    for path in paths:
        if autogen:
            gens = [max([int(re.search('logall_(\d+)', fname).group(1)) for fname in glob.glob(join(path, 'logall_*'))])]
        for gen in gens:
            logfile = join(path, 'logall_{}.txt'.format(gen))
            log = read_csvx(logfile, delimiter="\t")
            yield log


def getfitness(paths):
    for path in paths:
        fitpath = Path(path) / "fitnesslog.txt"
        try:
            fit = read_csvx(fitpath, delimiter='\t')
        except Exception:
            print("erreur, pas de run")
            continue
        yield fit


def getgenomes(paths, gens=None, forgiving=True):
    if gens is None:
        autogen = True
    else :
        autogen = False
    for cond in paths:
        if autogen:
            try:
                gens = [max([int(re.search('genomes_(\d+)', fname).group(1)) for fname in glob.glob(join(cond, 'genomes*'))])]
            except Exception as e:
                if forgiving:
                    continue
                else:
                    raise e
            stop = False
        for gen in gens:
            try:
                with open(cond + f'/genomes_{gen}.txt') as f:
                    dat = np.asarray(json.load(f))
                with open(glob.glob(cond + '/properties*')[0]) as f:
                    props = f.read()
                    maxcoop = float(re.search('maxCoop=(.*)', props).group(1))
                yield dat[:, 0] * maxcoop
            except FileNotFoundError as e:
                if forgiving:
                    continue
                else:
                    raise e

def getlastgen(paths, forgiving=True):
    for cond in paths:
        try:
            yield max([int(re.search('genomes_(\d+)', fname).group(1)) for fname in glob.glob(join(cond, 'genomes*'))])
        except Exception as e:
            if forgiving:
                continue
            else:
                raise e