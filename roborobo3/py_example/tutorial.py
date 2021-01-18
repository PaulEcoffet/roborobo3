from pyroborobo import Pyroborobo, Controller, PyWorldModel
import numpy as np


class TutorialController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion
    robot_index_offset = 1048576

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        self.rob = Pyroborobo.get()
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        # Simple avoidance
        self.set_translation(1)  # Let's go forward
        self.set_rotation(0)
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        camera_dist = self.world_model.camera_pixel_distance
        camera_max_range = self.world_model.maxdistcamera
        if (camera_dist[1] < camera_max_range  # if we see something on our right
                or camera_dist[2] < camera_max_range):  # or in front of us
            self.set_rotation(1)  # turn left
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our left
            self.set_rotation(-1)  # turn right
        # now let's get talkative
        if self.id == 0:
            print(f"I am {self}, {self.id}, at position {self.absolute_position} and orientation {self.absolute_orientation}")
            for i in range(self.nb_sensors):
                print(f"Sensor {i}:")
                print(f"\tdist: {self.world_model.camera_pixel_distance[i]}")
                print(f"\tid: {self.world_model.camera_objects_ids[i]}")
                print(f"\tis_wall: {self.get_wall_at(i)}")
                print(f"\tis_robot: {self.rob.is_id_robot(self.world_model.camera_objects_ids[i])}")
                is_robot = self.rob.is_id_robot(self.world_model.camera_objects_ids[i])
                is_wall = self.get_wall_at(i)
                if is_robot:
                    robid = self.get_robot_id_at(i)
                    print(f"\trobot id: {robid}")
                    print(f"\trobot object: {self.rob.robots[robid]}")
                    print(f"\trobot's controller: {self.get_robot_controller_at(i)}")
                    ctl = self.get_robot_controller_at(i)
                    print(f"\tThis robot is at {ctl.absolute_position} with orientation {ctl.absolute_orientation}.")
                elif not is_wall and self.world_model.camera_objects_ids[i] >= 0:  # then it's an object
                    print(f"\tphysical object instance: {self.get_object_at(i)}")


if __name__ == "__main__":
    rob = Pyroborobo.create("config/simple.properties",
                            controller_class=TutorialController,
                            world_model_class=PyWorldModel)
    rob.start()
    rob.update(10000)
    Pyroborobo.close()
