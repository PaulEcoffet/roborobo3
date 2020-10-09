from pyroborobo import PyController, Pyroborobo


class MyController(PyController):
    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to link the python object to its C++ counterpart
        PyController.__init__(self, world_model)
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.world_model.translation = 2  # Let's go forward
        self.world_model.rotation = 0  # and do not turn


if __name__ == "__main__":
    rob = Pyroborobo.create("config/template_wander_smallrobots.properties",
                            None, MyController, None, None, {}, {})
    # rob.start()
    rob.update(1000)
    Pyroborobo.close()
