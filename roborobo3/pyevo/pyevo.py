from pyevo.fitprop import FitPropEvolutionStrategy
import cma

def getES(type_, guess, sigma, popsize, bounds, maxiter, logpath):
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
    else:
        raise NotImplemented()
