from controllers import HungryController
from objects import SwitchObject, GateObject, ResourceObject

from pyroborobo import Pyroborobo, PyWorldModel

if __name__ == "__main__":
    rob = Pyroborobo.create("config/pywander_pyobj_resource.properties",
                            controller_class=HungryController,
                            world_model_class=PyWorldModel,
                            object_class_dict={'_default': ResourceObject, 'gate': GateObject, 'switch': SwitchObject})
    rob.start()
    rob.update(3000)
    Pyroborobo.close()
