from pyroborobo import Pyroborobo, Controller, PyWorldModel


class GatherController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to
        # link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        self.rob = Pyroborobo.get()
        self.rob_offset = self.rob.robot_index_offset
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.world_model.translation = 2  # Let's go forward
        self.world_model.rotation = 0
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        camera_dist = self.world_model.camera_pixel_distance
        camera_id = self.world_model.camera_objects_ids
        camera_max_range = self.world_model.maxdistcamera
        if camera_dist[1] < camera_max_range:  # if we see something on our right
            if camera_id[1] > self.rob_offset:  # and it's a robot
                self.world_model.rotation = -10  # go toward it
            else: # if it's an object
                self.world_model.rotation = 10  # turn left
        elif camera_dist[2] < camera_max_range:  # or in front of us
            if camera_id[2] > self.rob_offset:  # and it's an object
                self.world_model.rotation = 10  # avoid it
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our left
            if camera_id[3] > self.rob_offset:  # and it's a robot
                self.world_model.rotation = 10  # go toward the robot
            else:
                self.world_model.rotation = -10  # avoid it (turn right)


if __name__ == "__main__":
    rob = Pyroborobo.create("config/pywander.properties",
                            controller_class=GatherController,
                            world_model_class=PyWorldModel)
    rob.start()
    rob.update(1000)
    Pyroborobo.close()
