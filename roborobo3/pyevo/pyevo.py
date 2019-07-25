
from pyevo.fitprop import FitPropEvolutionStrategy
from pyevo.mulambdaes import MuLambdaEvolutionStrategy
from pyevo.oneone import OneOneEvolutionStrategy
import cma

def getES(type_, guess, sigma, popsize, bounds, maxiter, logpath, **kwargs):
    if type_ == 'cmaes':
        if callable(guess):
            guess = guess()
        return cma.CMAEvolutionStrategy(guess, 0.3,  # big sigma for init
                                      {'popsize': popsize,
                                      'BoundaryHandler': cma.s.ch.BoundTransform,
                                      'bounds': list(bounds),
                                      'verb_filenameprefix': logpath,
                                      'maxiter': maxiter
                                      })
    elif type_ == 'fitprop':
        normalmut = kwargs['normalmut']
        return FitPropEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                        logpath, normalmut=normalmut)
    elif type_ == 'mulambda':
        normalmut = kwargs['normalmut']
        mu = kwargs.get('mu', popsize//2)
        return MuLambdaEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                         logpath, mu=mu, normalmut=normalmut)
    elif type_ == 'oneone':
        mu = kwargs.get('mu', (9*popsize)//10)
        return OneOneEvolutionStrategy(guess, sigma, popsize, maxiter, bounds,
                                       logpath, mu=mu)
    else:
        raise NotImplemented()
