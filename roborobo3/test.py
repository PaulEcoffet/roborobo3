from pyroborobo import Pyroborobo, PyController
import numpy as np




class MyController(PyController):
    def __init__(self, wm):
        PyController.__init__(self, wm)  # NEEDED otherwise segfault, pybind limitation
        self.wm = wm

    def reset(self):
        pass

    def step(self):
        sensors = self.wm.getCameraSensorsDist()
        if np.all(sensors[2:7] > 0.5):
            self.wm.speed = 1
            self.wm.rotspeed = 0
        else:
            self.wm.speed = 0
            self.wm.rotspeed = 20


if __name__ == "__main__":
    # Use a dummy world observer that doesn't call any dynamic_cast
    rob = Pyroborobo("config/template_randomwalk.properties", "dummy", MyController, None, None, {})
    rob.start()
    stop = False
    while not stop:
        stop = rob.update(1000)  # update for 1000 time step before going back in python mode, except if quit early
    rob.close()