from pyroborobo import Controller, PyWorldModel, Pyroborobo


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


class HungryController(SimpleController):

    def __init__(self, wm):
        super().__init__(wm)
        self.rob = Pyroborobo.get()
        assert self.nb_sensors == 8, "SimpleController only works with 8 sensors"
        self.rotspeed = 0.5

    def step(self):
        self.set_translation(1)  # Let's go forward
        self.set_rotation(0)

        must_flee: bool = False  # have we encountered a wall and must prioritise avoiding obstacle

        camera_dist = self.world_model.camera_pixel_distance
        camera_max_range = self.world_model.maxdistcamera
        camera_ids = self.world_model.camera_objects_ids
        if camera_dist[1] < camera_max_range:  # if we see something on our right
            if self.get_object_at(1) != -1:  # And it is food
                self.set_rotation(-self.rotspeed)  # GO TOWARD IT
            else:
                self.set_rotation(self.rotspeed)  # flee it
                must_flee = True
        elif camera_dist[2] < camera_max_range:  # if we see something in front of us
            if self.get_object_at(2) != -1 and not must_flee:  # If we are not avoiding obstacle and it's food
                self.set_rotation(0)
            else:
                self.set_rotation(self.rotspeed)  # turn left
                must_flee = True
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our right
            if self.get_object_at(3) != -1 and not must_flee:
                self.set_rotation(self.rotspeed)  # turn left
            else:
                self.set_rotation(-self.rotspeed)
                must_flee = True

    def inspect(self, prefix):
        output = ""
        for i in range(self.nb_sensors):
            output += f"""sensor {i}:
    dist: {self.get_distance_at(i)}
    id: {self.world_model.camera_objects_ids[i]}
    nbobjs: {len(self.rob.objects)}
    is_object: {self.get_object_at(i)}
    is_robot: {self.get_robot_id_at(i)}
    is_wall: {self.get_wall_at(i)}\n\n"""
        return output