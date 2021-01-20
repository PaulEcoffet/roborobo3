from pyroborobo import Pyroborobo,  PyWorldModel, Controller, AgentObserver
import numpy as np
from scipy.stats import rankdata


def evaluate_network(input_, network):
    out = np.concatenate([[1], input_])
    for elem in network:
        out = np.tanh(out @ elem)
    return out


class EvolController(Controller):
    world_model: PyWorldModel


    def __init__(self, wm):
        Controller.__init__(self, wm)
        self.nb_hiddens = 10
        self.weights = [np.random.normal(0, 1, (self.nb_sensors + 1, self.nb_hiddens)), np.random.normal(0, 1, (self.nb_hiddens, 2))]

    def reset(self):
        pass

    def step(self):
        input = self.get_all_distances()
        out = evaluate_network(input, self.weights)
        self.set_translation(out[0])
        self.set_rotation(out[1])

    def get_flat_weights(self):
        all_elem = []
        for elem in self.weights:
            all_elem.append(elem.reshape(-1))
        return np.concatenate(all_elem)

    def set_weights(self, weights):
        i = 0
        for i, elem in enumerate(self.weights):
            shape = elem.shape
            size = elem.size
            self.weights[i] = np.array(weights[i:(i + size)]).reshape(shape)
            i += size


class EvolObserver(AgentObserver):
    world_model: PyWorldModel

    def __init__(self, wm):
        super().__init__(wm)
        self.fitness = 0

    def reset(self):
        self.fitness = 0

    def step_post(self):
        speed = self.controller.translation
        rotspeed = np.abs(self.controller.rotation)
        dists = np.asarray(self.controller.get_all_distances())
        if np.random.rand() < 0.0001:
            print(speed, rotspeed, dists)
            print(4*speed - 3*np.max(1 - dists) - np.abs(rotspeed))
        self.fitness += 4*speed - 3*np.max(1 - dists) - np.abs(rotspeed)


def get_weights(rob: Pyroborobo):
    weights = []
    for ctl in rob.controllers:
        weights.append(ctl.get_flat_weights())
    return weights


def get_fitnesses(rob: Pyroborobo):
    fitnesses = []
    for observer in rob.agent_observers:
        fitnesses.append(observer.fitness)
    return fitnesses


def fitprop(weights, fitnesses):
    adjust_fit = rankdata(fitnesses)
    print(adjust_fit)
    cumfit = np.cumsum(adjust_fit)
    normfit = cumfit / np.sum(cumfit)
    # select
    new_weights_i = np.random.choice(len(weights), len(weights), replace=True, p=normfit)
    new_weights = np.asarray(weights)[new_weights_i]
    print(new_weights_i)
    print(new_weights)
    # mutate
    new_weights_mutate = np.random.normal(new_weights, 0.1)
    return new_weights_mutate


def apply_weights(rob, weights):
    for ctl, weight in zip(rob.controllers, weights):
        ctl.set_weights(weight)



def reset_agent_observers(rob):
    for obs in rob.agent_observers:
        obs.reset()


def main():
    nbgen = 10000
    nbiterpergen = 200
    rob: Pyroborobo = Pyroborobo.create(
        "config/pywander.properties",
        controller_class=EvolController,
        world_model_class=PyWorldModel,
        agent_observer_class=EvolObserver,
    )

    rob.start()
    for igen in range(nbgen):
        print("*" * 10, igen, "*" * 10)
        stop = rob.update(nbiterpergen)
        if stop:
            break
        weights = get_weights(rob)
        fitnesses = get_fitnesses(rob)
        new_weights = fitprop(weights, fitnesses)
        apply_weights(rob, new_weights)
        reset_agent_observers(rob)


if __name__ == "__main__":
    main()
