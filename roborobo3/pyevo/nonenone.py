import numpy as np
from copy import copy, deepcopy
from json_tricks import dump

NOTHING = 0
UNIFORM = 1
NORMAL = 2


class NOneOneEvolutionStrategy:
    """Fitness Proportionate with ask and tell interface."""
    def __init__(self, genome_guess, mutation_rate, popsize, maxiter,
                 bounds, path, full_random_begin=False, normalmut=0.1, percentuni=0.1, **kwargs):
        self.iter = 0
        self.maxiter = maxiter
        self.mutation_rate = mutation_rate
        self.popsize = popsize
        self.log_every = 500
        self.percentuni = percentuni


        # boundaries are the same for all the dimension for now
        bounds = np.asarray(bounds)
        assert(np.all(bounds[0] < bounds[1]))
        self.minb = np.asarray(bounds[0])
        self.maxb = np.asarray(bounds[1])
        self.solutions = self._init_solutions(genome_guess, full_random_begin)
        if np.isscalar(normalmut):
            self.normalmut = np.asarray(np.tile(normalmut, np.asarray(self.solutions[0]).shape))
        else:
            self.normalmut = np.asarray(normalmut)

        if 'mutprob' in kwargs:
            self.mutprob = kwargs['mutprob']
            assert(len(self.mutprob) == len(self.minb))
        else:
            self.mutprob = np.repeat(self.mutation_rate, popsize)
        print(self.mutprob)
        assert(self.normalmut.shape[0] == np.asarray(self.solutions[0]).shape[0])
        self.lastfitnesses = np.repeat(1, self.popsize)
        self.prevfitnesses = np.repeat(1, self.popsize)
        self.prevsolutions = self.solutions
        self.logger = NOneOneLogger(self, path)
        self.nbweights = self.solutions.shape[1]

    def ask(self):
        # Compute the new solution, keep the best solution between the current eval and the previous one
        new_solutions = np.where(np.tile((self.lastfitnesses > self.prevfitnesses)[:, np.newaxis], self.nbweights), self.solutions, self.prevsolutions)

        # Normal transformation along all genes
        p_uni = self.percentuni
        p_normal = 1 - p_uni
        mutation_mask_p = np.tile(self.mutprob, (self.popsize, 1))
        mutation_mask = np.random.binomial(1, mutation_mask_p)
        mutation_mask[mutation_mask == 1] = np.random.choice([UNIFORM, NORMAL], size=np.sum(mutation_mask), p=[p_uni, p_normal])
        # Uniform transformation
        min_mask = np.tile(self.minb, (self.popsize, 1))
        max_mask = np.tile(self.maxb, (self.popsize, 1))
        std_mask = np.tile(self.normalmut, (self.popsize, 1))
        mutation_mask[np.where(std_mask == 0)] = NOTHING
        mutations = np.random.uniform(min_mask[mutation_mask == UNIFORM], max_mask[mutation_mask == UNIFORM])
        new_solutions[mutation_mask == UNIFORM] = mutations
        # normal transformation
        mutations = np.random.normal(new_solutions[mutation_mask == NORMAL], std_mask[mutation_mask == NORMAL])
        new_solutions[mutation_mask == NORMAL] = mutations
        np.clip(new_solutions, min_mask, max_mask, out=new_solutions)
        return deepcopy([solution for solution in new_solutions])

    def tell(self, solutions, fitnesses):
        self.prevsolutions = self.solutions
        self.solutions = np.asarray(solutions)
        self.prevfitnesses = self.lastfitnesses
        self.lastfitnesses = np.asarray(fitnesses)
        self.iter += 1

    def stop(self):
        return self.iter >= self.maxiter

    def _init_solutions(self, genome_guess, full_random_begin):
        if callable(genome_guess):
            out = np.array([genome_guess() for dummy in range(self.popsize)])
        elif full_random_begin:
            out = np.random.uniform(
                -1, 1, size=(self.popsize, len(self.minb)))  # TODO Hard coded guess
        else:
            if len(genome_guess.shape) == 1:
                nbelem = 1
            else:
                nbelem = genome_guess.shape[0]
            nbrep = self.popsize // nbelem # tile so that we have the whole population
            if self.popsize % nbelem != 0:  # We need to add the remaining
                nbrep += 1
            out = np.tile(genome_guess, (nbrep, 1))[:self.popsize] # we remove the overflow
        return out

    def disp(self):
        sorted_fitness = np.sort(self.lastfitnesses)
        size = len(sorted_fitness)
        print('{iter}\t{minfit}\t{qfit}\t{medfit}\t{q3fit}\t{maxfit}'.format(
            iter=self.iter,
            minfit=sorted_fitness[0],
            qfit=sorted_fitness[size//4],
            medfit=sorted_fitness[size//2],
            q3fit=sorted_fitness[(3*size)//4],
            maxfit=sorted_fitness[-1]
        ))

    @property
    def result(self):
        return [{'fit': fit, 'weights': weights} for weights, fit
                in zip(deepcopy(self.solutions), deepcopy(self.lastfitnesses))]


class NOneOneLogger:
    """Logger for the FitPropEvolutionStrategy"""
    def __init__(self, fitpropes, path):
        self.es = fitpropes
        self.path = path


    def add(self):
        with open(self.path + 'fit.txt', 'a') as f:
            sorted_fitness = np.sort(self.es.lastfitnesses)
            size = len(sorted_fitness)
            f.write('{iter}\t{minfit}\t{qfit}\t{medfit}\t{q3fit}\t{maxfit}\n'.format(
                iter=self.es.iter,
                minfit=sorted_fitness[0],
                qfit=sorted_fitness[size//4],
                medfit=sorted_fitness[size//2],
                q3fit=sorted_fitness[(3*size)//4],
                maxfit=sorted_fitness[-1]
            ))
        if self.es.iter % self.es.log_every == 0:
            with open(self.path + 'genome_{}.txt'.format(self.es.iter), 'a') as f:
                data = [{'weights': weights, 'fitness': fitness}
                        for weights, fitness in zip(self.es.solutions,
                                                    self.es.lastfitnesses)]
                dump(data, f, primitives=True)



    def plot(self):
        pass  # Not implemented yet
