# DEMO 1
import pyroborobo
from pyroborobo import Pyroborobo, PyWorldModel, Controller, CircleObject
import numpy as np


class ResourceObject(CircleObject):
    def __init__(self, id_, data):
        CircleObject.__init__(self, id_)  # Do not forget to call super constructor
        self.regrow_time = 100
        self.cur_regrow = 0
        self.triggered = False
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
        self.cur_regrow = self.regrow_time
        self.hide()
        self.unregister()

    def inspect(self, prefix=""):
        return f"""I'm a ResourceObject with id: {self.id}"""


class TalkingController(Controller):

    def __init__(self, wm: PyWorldModel):
        super().__init__(wm)
        self.rob = Pyroborobo.get()
        self.last_rob_met = -1
        self.last_obj_met = -1
        self.arena_mid_x, self.arena_mid_y = np.array(self.rob.arena_size) / 2

    def reset(self):
        pass

    def step(self):
        self.set_translation(1)
        self.set_rotation(np.random.choice([0, -0.5, 0.5]))
        for i in range(self.nb_sensors):
            rob = self.get_robot_controller_at(i)
            if rob:
                self.say_hello(rob)
            obj = self.get_object_instance_at(i)
            # obj = None
            if obj:
                self.last_obj_met = obj.id
        x, y = self.absolute_position
        color = [0, 0, 255]
        if x < self.arena_mid_x and y < self.arena_mid_y:
            color = [0, 0, 0]
        if x > self.arena_mid_x and y < self.arena_mid_y:
            color = [255, 0, 255]
        if x > self.arena_mid_x and y > self.arena_mid_y:
            color = [127, 127, 0]
        self.set_color(*color)

    def say_hello(self, rob):
        rob.receive_hello(self)

    def receive_hello(self, rob):
        self.last_rob_met = rob.id

    def inspect(self, prefix=""):
        return f"""
Hello, I'm robot {self.id}
The last object I saw was {self.last_obj_met}.
The last robot who said hello to me was {self.last_rob_met}
See you soon
"""


class SelectObject(CircleObject):

    def __init__(self, id, data):
        super().__init__(id)
        self.set_color(255, 165, 0)
        self.eaten = False
        self.data = data
        print(self.data)

    def reset(self):
        pass

    def step(self):
        if self.eaten:
            self.eaten = False
            self.relocate()
            self.show()

    def is_walked(self, robid):
        if robid == 9:
            self.hide()
            self.eaten = True


def main():
    rob = Pyroborobo.create("config/talking_robots.properties",
                            controller_class=TalkingController,
                            world_model_class=PyWorldModel,
                            object_class_dict={'_default': ResourceObject, 'select': SelectObject}
                            )
    rob.start()
    rob.update(1)
    print("Hello, welcome to demo 1")
    print("Press P to pause the simulation")
    print("Click on a robot to know more info about him")
    print("The orange object can only be activated by the robot #9")
    print("have fun")
    rob.update(1000)
    Pyroborobo.close()


if __name__ == "__main__":
    main()
