try:
    from pyevo.fitprop import FitPropEvolutionStrategy
    from pyevo.mulambdaes import MuLambdaEvolutionStrategy
except ImportError:
    from fitprop import FitPropEvolutionStrategy
    from mulambdaes import MuLambdaEvolutionStrategy
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
        return MuLambdaEvolutionStrategy(guess, 0.01, popsize, maxiter, bounds,
                                         logpath, full_random_begin=True,
                                         mu=mu)
    else:
        raise NotImplemented()
