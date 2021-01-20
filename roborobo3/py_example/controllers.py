from pyroborobo import Controller, PyWorldModel, Pyroborobo


class SimpleController(Controller):
    world_model: PyWorldModel  # Predeclare that world_model is a PyWorldModel for better code completion

    def __init__(self, world_model):
        # It is *mandatory* to call the super constructor before any other operation to link the python object to its C++ counterpart
        Controller.__init__(self, world_model)
        print("I'm a Python controller")

    def reset(self):
        print("I'm initialised")

    def step(self):  # step is called at each time step
        self.set_translation(1)  # Let's go forward
        self.set_rotation(0)
        # Now, our world_model object is a PyWorldModel, we can have access to camera_* properties
        if (self.get_distance_at(1) < 1 # if we see something on our left
            or self.get_distance_at(2) < 1): # or in front of us
            self.set_rotation(0.5) # turn right
        elif self.get_distance_at(3) < 1: # Otherwise, if we see something on our right
            self.set_rotation(-0.5) # turn left


class HungryController(SimpleController):

    def __init__(self, wm):
        super().__init__(wm)
        self.rob = Pyroborobo.get()

    def step(self):
        self.set_translation(1)  # Let's go forward
        self.set_rotation(0)

        must_flee: bool = False  # have we encountered a wall and must prioritise avoiding obstacle

        camera_dist = self.world_model.camera_pixel_distance
        camera_max_range = self.world_model.maxdistcamera
        camera_ids = self.world_model.camera_objects_ids
        if camera_dist[1] < camera_max_range:  # if we see something on our right
            if self._is_food(camera_ids[1]):  # And it is food
                self.set_rotation(-1)  # GO TOWARD IT
            else:
                self.set_rotation(1)  # flee it
                must_flee = True
        elif camera_dist[2] < camera_max_range:  # if we see something in front of us
            if self._is_food(camera_ids[2]) and not must_flee:  # If we are not avoiding obstacle and it's food
                self.set_rotation(0)
            else:
                self.set_rotation(1)  # turn left
                must_flee = True
        elif camera_dist[3] < camera_max_range:  # Otherwise, if we see something on our right
            if self._is_food(camera_ids[3]) and not must_flee:
                self.set_rotation(1)  # turn left
            else:
                self.set_rotation(-1)
                must_flee = True

    def _is_food(self, id_):
        return not self.rob.is_id_robot(id_)
