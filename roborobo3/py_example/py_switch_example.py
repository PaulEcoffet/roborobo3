from pyroborobo import Pyroborobo, PyWorldModel
from custom.controllers import SimpleController
from custom.objects import SwitchObject, GateObject

if __name__ == "__main__":
    rob = Pyroborobo.create("config/pywander_pyobj.properties",
                            controller_class=SimpleController,
                            world_model_class=PyWorldModel,
                            object_class_dict={'gate': GateObject, 'switch': SwitchObject})
    rob.start()
    rob.update(3000)
    Pyroborobo.close()
