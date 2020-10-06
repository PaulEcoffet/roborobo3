from typing import List

import numpy as np
from _pyroborobo import _PyWorldModel

# Imported from roborobo definitions in RobotWorldModel.h

_SENSOR_REGISTERID = 0
_SENSOR_SOURCENORM = 1
_SENSOR_SOURCEANGLE = 2
_SENSOR_TARGETNORM = 3
_SENSOR_TARGETANGLE = 4
_SENSOR_DISTANCEVALUE = 5
_SENSOR_OBJECTVALUE = 6


class PyWorldModel(_PyWorldModel):
    """
    Attributes
    ----------
    f_sensors: List[int]
        sensor indices exactly in front of the robot

    fr_sensors: List[int]
        sensor indices at the front right of the robot

    fl_sensors: List[int]
        sensor indices at the front left of the robot

    rr_sensors: List[int]
        sensor indices at the rear right of the robot

    rl_sensors: List[int]
        sensor indices at the rear left of the robot

    """

    def __init__(self):
        _PyWorldModel.__init__(self)

    def reset(self):
        self._camerasensors = np.asarray(self._get_camera_sensors())
        self.maxdistcamera = self._get_max_dist_camera()
        cam_angles = self.camera_angles
        self.f_sensors = [i for i, angle in enumerate(cam_angles) if -0.0001 < angle < 0.00001]
        self.fr_sensors = [i for i, angle in enumerate(cam_angles) if -np.pi / 2 < angle < 0]
        self.fl_sensors = [i for i, angle in enumerate(cam_angles) if 0 < angle < np.pi / 2]
        self.rl_sensors = [i for i, angle in enumerate(cam_angles) if np.pi / 2 < angle < np.pi]
        self.rr_sensors = [i for i, angle in enumerate(cam_angles) if -np.pi < angle < -np.pi / 2]

    @property
    def camera_objects_ids(self):
        return np.asarray(self._camerasensors[:, _SENSOR_OBJECTVALUE], dtype=int)

    @property
    def camera_pixel_distance(self):
        return self._camerasensors[:, _SENSOR_DISTANCEVALUE]

    @property
    def camera_normalized_distance(self):
        """numpy view of the normalized distances of the objects seen by the robot sensors. If the distance is 1,
no object is seen by the sensor. If the distance is 0, the object is right next to the robot."""
        return self.camera_pixel_distance / self.maxdistcamera

    @property
    def camera_angles(self):
        """numpy view of the angles of the sensors target relative to the center of the robots in radian.
0 -> exactly in front. [-π/2, 0] -> to the front right. [-π, -π/2] -> to the rear right.
[0, π/2] -> front left, [π/2, π] -> rear left.
"""
        return self._camerasensors[:, _SENSOR_TARGETANGLE]
