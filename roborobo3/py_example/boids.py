from pyroborobo import Pyroborobo, Controller, PyWorldModel
import numpy as np


def principal_value(deg):
    deg_mod = np.mod(deg, 360)
    if deg_mod > 180:
        return deg_mod - 360
    else:
        return deg_mod


def angle_diff(x, y):
    return principal_value(x - y)


class BoidsController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to
        # link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        self.rob = Pyroborobo.get()
        self.rob_offset = self.rob.robot_index_offset
        self.rotspeed = 0.2
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def get_orientation(self, other_id):
        robot_id = other_id - self.rob_offset
        return self.rob.world_models[robot_id].absolute_orientation

    def step(self):  # step is called at each time step
        self.world_model.translation = 2  # Let's go forward
        self.world_model.rotation = 0
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        camera_dist = self.world_model.camera_pixel_distance
        camera_id = self.world_model.camera_objects_ids
        camera_max_range = self.world_model.maxdistcamera
        camera_angle_rad = self.world_model.camera_angles
        camera_angle = camera_angle_rad * 180
        repulse_radius = np.min([4, camera_max_range//4])
        orientation_radius = camera_max_range // 2
        own_orientation = self.world_model.absolute_orientation

        for i, dist in enumerate(camera_dist):
            if dist < repulse_radius:
                if camera_angle[i] != 0:
                    self.world_model.rotation = -camera_angle[i] * self.rotspeed
                else:
                    self.world_model.rotation = 180 * self.rotspeed
                self.world_model.translation = 1
                break
            elif dist < orientation_radius and camera_id[i] > self.rob_offset:
                orient_angle = angle_diff(own_orientation, self.get_orientation(int(camera_id[i])))
                self.world_model.rotation = -orient_angle * self.rotspeed
                break
            elif dist < camera_max_range and camera_id[i] > self.rob_offset:
                self.world_model.rotation = camera_angle[i] * self.rotspeed
                break


if __name__ == "__main__":
    rob = Pyroborobo.create("config/pyboids.properties",
                            controller_class=BoidsController,
                            world_model_class=PyWorldModel)
    rob.start()
    rob.update(100000)
    Pyroborobo.close()
