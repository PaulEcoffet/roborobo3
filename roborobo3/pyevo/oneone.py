import numpy as np
from copy import copy, deepcopy
from json_tricks import dump


class OneOneEvolutionStrategy():
    """MuLambda ES with ask and tell interface."""
    def __init__(self, genome_guess, mutation_rate, popsize, maxiter,
                 bounds, path, full_random_begin=False, mu=1):
        self.iter = 0
        self.maxiter = maxiter
        self.mutation_rate = mutation_rate
        self.popsize = popsize
        self.mu = mu

        # boundaries are the same for all the dimension for now
        assert(bounds[0] < bounds[1])
        self.minb = bounds[0]
        self.maxb = bounds[1]
        self.solutions = self._init_solutions(genome_guess, full_random_begin)
        self.lastfitnesses = np.repeat(1, self.popsize)
        self.logger = MuLambdaLogger(self, path)

    def ask(self):
        if self.iter == 0:
            return self.solutions
        fitnesses = self.lastfitnesses
        fitnesses[:self.mu] = fitnesses[:self.mu].mean()
        fitnesses[self.mu:] = fitnesses[self.mu:].mean()
        new_pop_index = np.argmax(fitnesses)
        if new_pop_index >= self.mu:
            print('Invasion de mutants')
        parent = self.solutions[new_pop_index].copy()
        child = parent.copy()
        igen = np.random.choice(child.shape[0], size=1, replace=False)
        # Yes this is super weird sorry
        child[igen] = np.random.uniform(self.minb, self.maxb, size=1)
        new_solutions = np.concatenate(
            (
                np.tile(parent, (self.mu, 1)),
                np.tile(child, (self.popsize - self.mu, 1))
            ))
        assert(len(new_solutions) == self.popsize)
        return new_solutions

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
                self.minb, self.maxb, size=(self.popsize, nb_weights))
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


class MuLambdaLogger:
    """Logger for the MuLambdaEvolutionStrategy"""
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



    def plot(self):
        pass  # Not implemented yet
