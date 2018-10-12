
from pyevo.fitprop import FitPropEvolutionStrategy
from pyevo.mulambdaes import MuLambdaEvolutionStrategy
from pyevo.oneone import OneOneEvolutionStrategy
import cma

def getES(type_, guess, sigma, popsize, bounds, maxiter, logpath, **kwargs):
    if type_ == 'cmaes':
        return cma.CMAEvolutionStrategy(guess, 0.3,  # big sigma for init
                                      {'popsize': popsize,
                                      'BoundaryHandler': cma.s.ch.BoundTransform,
                                      'bounds': bounds,
                                      'verb_filenameprefix': logpath,
                                      'maxiter': maxiter
                                      })
    elif type_ == 'fitprop':
        return FitPropEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                        logpath, full_random_begin=True)
    elif type_ == 'mulambda':
        mu = kwargs.get('mu', popsize//2)
        return MuLambdaEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                         logpath, full_random_begin=True,
                                         mu=mu)
    elif type_ == 'oneone':
        mu = kwargs.get('mu', (9*popsize)//10)
        return OneOneEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                       logpath, full_random_begin=True, mu=mu)
    else:
        raise NotImplemented()
