import numpy as np
from copy import copy, deepcopy
from json_tricks import dump


class FitPropEvolutionStrategy():
    """Fitness Proportionate with ask and tell interface."""
    def __init__(self, genome_guess, mutation_rate, popsize, maxiter,
                 bounds, path, full_random_begin=False):
        self.iter = 0
        self.maxiter = maxiter
        self.mutation_rate = mutation_rate
        self.popsize = popsize
        self.log_every = 500

        # boundaries are the same for all the dimension for now
        assert(bounds[0] < bounds[1])
        self.minb = bounds[0]
        self.maxb = bounds[1]
        self.solutions = self._init_solutions(genome_guess, full_random_begin)
        self.lastfitnesses = np.repeat(1, self.popsize)
        self.logger = FitPropLogger(self, path)

    def ask(self):
        fitnesses = np.clip(self.lastfitnesses, 1, None)
        new_pop_index = np.random.choice(len(self.solutions),
                                         self.popsize,
                                         p=(fitnesses/np.sum(fitnesses)))
        new_solutions = self.solutions[new_pop_index]
        normal_trans = False
        # Normal transformation along all genes
        if normal_trans:
            new_solutions = np.random.normal(new_solutions, self.mutation_rate)
        else:  # pick few genes and uniform transformation on them.
            p = self.mutation_rate
            mutation_mask = np.random.choice([True, False], size=new_solutions.shape, p=[p, 1 - p])
            print("mask type :", mutation_mask.dtype)
            mutations = np.random.uniform(self.minb, self.maxb, size=mutation_mask.sum())
            new_solutions[mutation_mask] = mutations
        np.clip(new_solutions, self.minb, self.maxb, out=new_solutions)
        return deepcopy([solution for solution in new_solutions])

    def tell(self, solutions, fitnesses):
        self.solutions = np.asarray(solutions)
        self.lastfitnesses = np.asarray(fitnesses)
        self.iter += 1

    def stop(self):
        return self.iter >= self.maxiter

    def _init_solutions(self, genome_guess, full_random_begin):
        nb_weights = len(genome_guess)
        if full_random_begin:
            out = np.random.uniform(
                -1, 1, size=(self.popsize, nb_weights))  # TODO Hard coded guess
        else:
            out = np.tile(genome_guess, (self.popsize, 1))
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
