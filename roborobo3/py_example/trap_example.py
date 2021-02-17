# Demo 2

from pyroborobo import Pyroborobo, PyWorldModel
from custom.objects import SwitchObject, GateObject
from custom.controllers import SimpleController

def main():
    rob = Pyroborobo.create("config/trap.properties",
                            controller_class=SimpleController,
                            world_model_class=PyWorldModel,
                            object_class_dict={'gate': GateObject, 'switch': SwitchObject})
    rob.start()
    rob.update(3000)
    Pyroborobo.close()


if __name__ == "__main__":
    main()