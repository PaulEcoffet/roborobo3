# Demo 4 : Ball

from pyroborobo import Pyroborobo, PyWorldModel, WorldObserver, MovableObject
from custom.controllers import HungryController
import numpy as np


class BallObject(MovableObject):

    def __init__(self, id_, data):
        MovableObject.__init__(self, id_)
        self.data = data

    def step(self):
        super().step()
        if np.random.random() < 0.001:
            self.hide()
            self.relocate()
            self.show()

    def is_pushed(self, id_, speed):
        super().is_pushed(id_, speed)
        print(f"I'm kicked by {id_}")

    def inspect(self, prefix):
        return f"[INSPECT] Ball #{self.id}\n"


def main():
    rob: Pyroborobo = Pyroborobo.create("config/ball.properties",
                            controller_class=HungryController,
                            world_model_class=PyWorldModel,
                            object_class_dict={"ball": BallObject})
    # Do not forget to set gMovableObjects in properties
    rob.start()
    print(rob.objects)
    rob.update(10000)
    Pyroborobo.close()


if __name__ == "__main__":
    main()


