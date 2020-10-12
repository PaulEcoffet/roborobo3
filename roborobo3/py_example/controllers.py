from pyroborobo import Pyroborobo, PyController, PyWorldModel


class SimpleController(PyController):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to link the python object to its C++ counterpart
        PyController.__init__(self, world_model)
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.world_model.translation = 1  # Let's go forward
        self.world_model.rotation = 0
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        camera_dist = self.world_model.camera_pixel_distance
        camera_max_range = self.world_model.maxdistcamera
        if (camera_dist[1] < camera_max_range  # if we see something on our left
                or camera_dist[2] < camera_max_range):  # or in front of us
            self.world_model.rotation = 10  # turn right
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our right
            self.world_model.rotation = -10  # turn left
