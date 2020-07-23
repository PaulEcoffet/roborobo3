import numpy as np
from copy import copy, deepcopy
from json_tricks import dump

NOTHING = 0
UNIFORM = 1
NORMAL = 2


class FitPropEvolutionStrategy():
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
        self.nbweights = len(self.minb)
        self.randomguess = kwargs['randomguess']
        self.init_min = kwargs['init_min']
        self.init_max = kwargs['init_max']
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
        self.logger = FitPropLogger(self, path)

    def ask(self):
        fitnesses = np.asarray(self.lastfitnesses)
        assert(not np.any(np.isnan(fitnesses)))
        if np.min(fitnesses) <= 0:
            fitnesses -= (np.min(self.lastfitnesses) - 1)
        assert(np.all(fitnesses >= 0) and np.any(fitnesses > 0))  # all positive and at least one not nul
        new_pop_index = np.random.choice(len(self.solutions),
                                         self.popsize,
                                         p=(fitnesses/np.sum(fitnesses)))
        new_solutions = self.solutions[new_pop_index]
        new_solutions = self._mutate(new_solutions)
        return deepcopy([solution for solution in new_solutions])

    def tell(self, solutions, fitnesses):
        self.solutions = np.asarray(solutions)
        self.lastfitnesses = np.asarray(fitnesses)
        self.iter += 1

    def stop(self):
        return self.iter >= self.maxiter

    def _mutate(self, solution):
        prev_shape = solution.shape
        solution = np.copy(solution.reshape(-1, self.nbweights))
        localpopsize = solution.shape[0]
        # Normal transformation along all genes
        p_uni = self.percentuni
        p_normal = 1 - p_uni
        mutation_mask_p = np.tile(self.mutprob, (localpopsize, 1))
        mutation_mask = np.random.binomial(1, mutation_mask_p)
        mutation_mask[mutation_mask == 1] = np.random.choice([UNIFORM, NORMAL], size=np.sum(mutation_mask), p=[p_uni, p_normal])
        # Uniform transformation
        min_mask = np.tile(self.minb, (localpopsize, 1))
        max_mask = np.tile(self.maxb, (localpopsize, 1))
        std_mask = np.tile(self.normalmut, (localpopsize, 1))
        mutation_mask[np.where(std_mask == 0)] = NOTHING
        mutations = np.random.uniform(min_mask[mutation_mask == UNIFORM], max_mask[mutation_mask == UNIFORM])
        solution[mutation_mask == UNIFORM] = mutations
        # normal transformation
        mutations = np.random.normal(solution[mutation_mask == NORMAL], std_mask[mutation_mask == NORMAL])
        solution[mutation_mask == NORMAL] = mutations
        np.clip(solution, min_mask, max_mask, out=solution)
        return solution.reshape(prev_shape)

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
            for i in range(out.shape[0]):
                rand = np.random.uniform(self.init_min, self.init_max, self.nbweights)
                out[i, self.randomguess] = rand[self.randomguess]
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


class FitPropLogger:
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
