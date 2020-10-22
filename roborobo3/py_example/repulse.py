from pyroborobo import Pyroborobo, Controller, PyWorldModel
import numpy as np


class RepulseController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to
        # link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        self.rob = Pyroborobo.get()
        self.orientation = 1
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.world_model.translation = 2  # Let's go forward

        if np.random.random() < 0.01:  # Change orientation every 100 frames or so
            self.orientation = -self.orientation

        self.world_model.rotation = 3 * self.orientation
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        camera_dist = self.world_model.camera_pixel_distance
        camera_id = self.world_model.camera_objects_ids
        camera_max_range = self.world_model.maxdistcamera
        if camera_dist[1] < camera_max_range:  # if we see something on our right
            if self.rob.is_id_robot(camera_id[1]):  # and it's a robot
                self.world_model.rotation = 10  # go avoid it
            else:  # if it's an object
                self.world_model.rotation = 10  # turn left
        elif camera_dist[2] < camera_max_range:  # or in front of us
            self.world_model.rotation = 10  # avoid it
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our left
            if self.rob.is_id_robot(camera_id[3]):  # and it's a robot
                self.world_model.rotation = -10  # go toward the robot
            else:
                self.world_model.rotation = -10  # avoid it (turn right)


if __name__ == "__main__":
    rob = Pyroborobo.create("config/simple.properties",
                            controller_class=RepulseController,
                            world_model_class=PyWorldModel)
    rob.start()
    rob.update(10000)
    Pyroborobo.close()
