import numpy as np
from copy import copy, deepcopy
from json_tricks import dump


NOTHING = 0
UNIFORM = 1
NORMAL = 2


class MuLambdaEvolutionStrategy():
    """MuLambda ES with ask and tell interface."""
    def __init__(self, genome_guess, mutation_rate, popsize, maxiter,
                 bounds, path, full_random_begin=False, mu=1):
        self.iter = 0
        self.maxiter = maxiter
        self.mutation_rate = mutation_rate
        self.popsize = popsize
        self.normalmut = 0.1 # TODO SHOULDNT BE FIXED
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
        new_pop_index = np.argpartition(
            fitnesses, -self.mu)[-self.mu:]
        parents = self.solutions[new_pop_index].copy()
        true_parent_indexes = np.random.choice(
            len(parents), size=self.popsize - self.mu)
        children = parents[true_parent_indexes].copy()
        p = self.mutation_rate
        p_uni = 0.1 * p
        p_normal = p - p_uni
        mutation_mask = np.random.choice([NOTHING, UNIFORM, NORMAL], size=children.shape, p=[1 - p, p_uni, p_normal])
        # Uniform transformation
        min_mask = np.tile(self.minb, (self.popsize - self.mu, 1))
        max_mask = np.tile(self.maxb, (self.popsize - self.mu, 1))
        mutations = np.random.uniform(min_mask[mutation_mask == UNIFORM], max_mask[mutation_mask == UNIFORM])
        children[mutation_mask == UNIFORM] = mutations
        # normal transformation
        mutations = np.random.normal(0, 0.05, size=(mutation_mask == NORMAL).sum())
        children[mutation_mask == NORMAL] += mutations

        # force coop mut (UGLY)
        #### UGLY UGLY UGLY ####
        children[:children.shape[0]//2 , 0] += np.random.normal(0, self.normalmut, size=children.shape[0]//2)
        ########################

        np.clip(children, min_mask, max_mask, out=children)
        new_solutions = np.concatenate((parents, children))
        return new_solutions

    def tell(self, solutions, fitnesses):
        self.solutions = np.asarray(solutions)
        self.lastfitnesses = np.asarray(fitnesses)
        self.iter += 1

    def stop(self):
        return self.iter >= self.maxiter

    def _init_solutions(self, genome_guess, full_random_begin):
        try:
            nb_weights = len(genome_guess)
        except TypeError:
            return [genome_guess() for i in range(self.popsize)]
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
