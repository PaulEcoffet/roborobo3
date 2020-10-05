from pyroborobo import _PyWorldModel


class PyWorldModel(_PyWorldModel):
    def __init__(self):
        _PyWorldModel.__init__(self)
        self.camerasensors = self._get_camera_sensors()
