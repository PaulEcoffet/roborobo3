from pyroborobo import Pyroborobo, WorldObserver, SquareObject


class CreateObjectWorldObserver(WorldObserver):
    def __init__(self, world):
        super().__init__(world)
        self.rob = Pyroborobo.get()

    def init_post(self):
        print("coucou")
        for i in range(16):
            squareobj = SquareObject(-1)
            squareobj.soft_width = 0
            squareobj.soft_height = 0
            squareobj.solid_width = 10
            squareobj.soft_height = 10
            squareobj.set_coordinates(10 + 10 * i, 10)
            squareobj = self.rob.add_object(squareobj)
            squareobj.register()


def main():
    rob = Pyroborobo.create("config/boids.properties",
                            world_observer_class=CreateObjectWorldObserver,
                            override_conf_dict={"gInitialNumberOfRobots": "2"}
                            )
    rob.start()
    rob.update(1000)
    Pyroborobo.close()


if __name__ == "__main__":
    main()
