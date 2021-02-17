# Demo 4 : Ball

from pyroborobo import Pyroborobo, PyWorldModel
from custom.controllers import HungryController

def main():
    rob: Pyroborobo = Pyroborobo.create("config/ball.properties",
                            controller_class=HungryController,
                            world_model_class=PyWorldModel)
    # Do not forget to set gMovableObjects in properties
    rob.start()
    print(rob.objects)
    rob.update(10000)
    Pyroborobo.close()


if __name__ == "__main__":
    main()