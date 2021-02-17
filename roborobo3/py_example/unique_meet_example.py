# Demo 3
# each agent count the unique partners they met

from pyroborobo import Pyroborobo, Controller, WorldObserver, PyWorldModel
from custom.controllers import SimpleController

class CounterController(SimpleController):
    def __init__(self, wm):
        super().__init__(wm)
        self.agents_met = set()

    def step(self):
        super().step()
        for i in range(self.nb_sensors):
            rob = self.get_robot_id_at(i)
            if rob > 0:
                self.agents_met.add(rob)

    def inspect(self, prefix=""):
        return f"I'm {self.id}, and I met {len(self.agents_met)}.\n" \
               f"They are {self.agents_met}.\n"


class CounterWorldObserver(WorldObserver):
    def __init__(self, world):
        super().__init__(world)
        print("coucou")
        self.rob = Pyroborobo.get()

    def step_post(self):
        super().step_pre()
        print("post post")
        for ctlrob in self.rob.controllers:
            print(ctlrob.inspect())


def main():
    rob = Pyroborobo.create(
        "config/unique_meet.properties",
        controller_class=CounterController,
        world_observer_class=CounterWorldObserver,
        world_model_class=PyWorldModel,
        override_conf_dict={"gNbOfInitialRobots": 30}
    )
    rob.start()
    rob.update(3000)
    Pyroborobo.close()

if __name__ == "__main__":
    main()