from pyroborobo import Pyroborobo, Controller
import numpy as np

class GoToClosestLandMarkController(Controller):
    def __init__(self, wm):
        super().__init__(wm)

    def reset(self):
        pass

    def step(self):
        orient = self.get_closest_landmark_orientation()
        self.set_rotation(np.clip(orient, -1, 1))
        self.set_translation(1)


def change_landmark_positions(rob: Pyroborobo):
    arena_size = np.asarray(rob.arena_size)
    for landmark in rob.landmarks:
        lbound = np.array([landmark.radius + 8] * 2)
        ubound = arena_size - (landmark.radius + 8)
        x, y = np.random.randint(lbound, ubound)
        landmark.hide()
        landmark.set_coordinates(x, y)
        landmark.show()


def main():
    rob = Pyroborobo.create("config/landmarks.properties",
                      controller_class=GoToClosestLandMarkController)
    rob.start()
    for i in range(10):
        rob.update(400)
        change_landmark_positions(rob)
    Pyroborobo.close()


if __name__ == "__main__":
    main()
