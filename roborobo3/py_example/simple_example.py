from pyroborobo import Pyroborobo, PyWorldModel
from custom.controllers import SimpleController

if __name__ == "__main__":
    rob = Pyroborobo.create("config/pywander.properties",
                            controller_class=SimpleController,
                            world_model_class=PyWorldModel)
    rob.start()
    rob.update(1000)
    Pyroborobo.close()
