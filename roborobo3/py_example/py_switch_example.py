from pyroborobo import Pyroborobo, PyWorldModel, Controller, CircleObject, SquareObject


class SimpleController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        assert self.nb_sensors == 8, "SimpleController only works with 8 sensors"
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.set_translation(1)  # Let's go forward
        self.set_rotation(0)
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        if (self.get_distance_at(1) < 1  # if we see something on our left
                or self.get_distance_at(2) < 1):  # or in front of us
            self.set_rotation(0.5)  # turn right
        elif self.get_distance_at(3) < 1:  # Otherwise, if we see something on our right
            self.set_rotation(-0.5)  # turn left


class SwitchObject(CircleObject):
    def __init__(self, id, data):
        CircleObject.__init__(self, id)  # Do not forget to call super constructor
        self.regrow_time = data['regrowTimeMax']
        self.cur_regrow = 0
        self.triggered = False
        self.gate_id = data['sendMessageTo']
        self.rob = Pyroborobo.get()  # Get pyroborobo singleton

    def reset(self):
        self.show()
        self.register()
        self.triggered = False
        self.cur_regrow = 0

    def step(self):
        if self.triggered:
            self.cur_regrow -= 1
            if self.cur_regrow <= 0:
                self.show()
                self.register()
                self.triggered = False

    def is_walked(self, rob_id):
        self.triggered = True
        self.rob.objects[self.gate_id].open()
        self.cur_regrow = self.regrow_time
        self.hide()
        self.unregister()

    def inspect(self, prefix=""):
        return "I'm a switch!"


class GateObject(SquareObject):
    def __init__(self, id, data):
        SquareObject.__init__(self, id)
        self.triggered = False
        self.regrow_time = data['regrowTimeMax']
        self.cur_regrow = 0

    def reset(self):
        self.show()
        self.register()
        self.triggered = False
        self.cur_regrow = 0

    def step(self):
        if self.triggered:
            self.cur_regrow -= 1
            if self.cur_regrow <= 0:
                self.show()
                self.register()
                self.triggered = False

    def open(self):
        self.triggered = True
        self.hide()
        self.unregister()
        self.cur_regrow = self.regrow_time

    def inspect(self, prefix=""):
        return "I'm a gate!"



if __name__ == "__main__":
    rob = Pyroborobo.create("config/pywander_pyobj.properties",
                            controller_class=SimpleController,
                            world_model_class=PyWorldModel,
                            object_class_dict={'gate': GateObject, 'switch': SwitchObject})
    rob.start()
    rob.update(3000)
    Pyroborobo.close()
