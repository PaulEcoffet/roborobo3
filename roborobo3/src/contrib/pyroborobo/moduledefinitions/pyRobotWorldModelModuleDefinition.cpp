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
    py::class_<sensor_array, std::shared_ptr<sensor_array>>(m, "sensor_array", py::buffer_protocol())
            .def_buffer([](sensor_array &s) {
                return py::buffer_info(
                        s.data(),
                        sizeof(double),
                        py::format_descriptor<double>::format(),
                        s.num_dimensions(),
                        {s.shape()[0], s.shape()[1]},
                        {s.strides()[0] * sizeof(double), s.strides()[1] * sizeof(double)},
                        true
                );
            });


    py::class_<RobotWorldModel, PyWorldModel, std::shared_ptr<RobotWorldModel>>(m, "_PyWorldModel")
            .def(py::init_alias<>())
            .def("reset", &RobotWorldModel::reset)
            .def("_get_max_dist_camera", [](PyWorldModel &self) { return gSensorRange; })
            .def("_get_camera_sensors", [](PyWorldModel &self) -> const sensor_array & {
                     return self.getCameraSensors();
                 }, py::return_value_policy::reference,
                 R"doc(
Return the buffer view of the camera. Should not be called, use
)doc")
            .def("get_ground_sensor_values", [](PyWorldModel &self) { return self.getRobotGroundSensors(); },
                 R"doc(
Returns a tuple (red, green, blue) from the ground sensor of the robot
)doc")
            .def_property("alive", &RobotWorldModel::isAlive, &RobotWorldModel::setAlive, R"doc(
State if the robot is alive
)doc")
            .def_readwrite("translation", &RobotWorldModel::_desiredTranslationalValue, R"doc(
Robot's desired translation speed in pixels.
)doc")
            .def_readwrite("rotation", &RobotWorldModel::_desiredRotationalVelocity, R"doc(
Robot's desired rotation speed in degrees.
)doc")
            .def_readwrite("fitness", &RobotWorldModel::_fitnessValue, R"doc(
Robot's actual fitness
)doc");
}