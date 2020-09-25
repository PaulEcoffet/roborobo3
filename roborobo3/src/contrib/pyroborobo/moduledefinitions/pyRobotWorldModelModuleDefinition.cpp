//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyRobotWorldModelModuleDefinition.h"
#include <pybind11/pybind11.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <contrib/pyroborobo/PyWorldModel.h>

namespace py = pybind11;

void addPyRobotWorldModelBinding(py::module &m)
{
    py::class_<RobotWorldModel, PyWorldModel>(m, "PyWorldModel")
            .def(py::init<>())
            .def("get_camera_sensors_dist", [](PyWorldModel &self)
                 { return self.getCameraSensorsDist(); },
                 R"doc(
Returns a numpy array of the normalized distances of the objects seen by the robot sensors. If the distance is 1,
no object is seen by the sensor. If the distance is 0, the object is completely adjacent to the robot.

Returns
-------
A numpy.array of the objects seen by the robot's sensors.
)doc")
            .def("get_ground_sensor_values", [](PyWorldModel &self)
                 { return self.getRobotGroundSensors(); },
                 R"doc(
Returns a tuple (red, green, blue) from the ground sensor of the robot
)doc")
            .def("get_camera_object_ids", [](PyWorldModel &self)
                 { return self.getCameraObjectIds(); },
                 R"doc(
Returns a numpy array of the IDs of the objects seen by the robot
)doc")
            .def_property("alive", &RobotWorldModel::isAlive, &RobotWorldModel::setAlive)
            .def_readwrite("translation", &RobotWorldModel::_desiredTranslationalValue)
            .def_readwrite("rotation", &RobotWorldModel::_desiredRotationalVelocity)
            .def_readwrite("fitness", &RobotWorldModel::_fitnessValue);
}
