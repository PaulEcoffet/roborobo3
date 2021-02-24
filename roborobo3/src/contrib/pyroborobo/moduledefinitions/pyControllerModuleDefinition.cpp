//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyWorldObserverModuleDefinition.h"
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotWorldModelModuleDefinition.h>
#include <contrib/pyroborobo/SquareObjectTrampoline.h>
#include <contrib/pyroborobo/CircleObjectTrampoline.h>
#include <core/RoboroboMain/roborobo.h>
#include <contrib/pyroborobo/ControllerTrampoline.h>
#include <contrib/pyroborobo/RobotWorldModelTrampoline.h>
#include <core/Agents/Robot.h>
#include <core/World/World.h>
#include <core/Observers/WorldObserver.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <core/Controllers/Controller.h>
#include <pyroborobo/pyroborobo.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "contrib/pyroborobo/ModuleDefinitions/pyControllerModuleDefinition.h"
#include "contrib/pyroborobo/PyArrayHelpers.h"

namespace py = pybind11;

void addPyControllerBinding(py::module &m)
{
    pybind11::class_<Controller, ControllerTrampoline, std::shared_ptr<Controller>>(m, "Controller",
                                                                                    R"doc(
Class to extend a Roborobo Controller in python.

.. warning::
    If the __init__ is overridden, it is absolutely necessary to call the PyController constructor by doing :

    .. code-block:: python

        def __init__(self, world_model):
            PyController.__init__(self, world_model)

    Not doing so leads to cryptic errors due to the interface between python and c++.

    It is also necessary to override `step` and `reset`. Not doing so leads to cryptic errors.


)doc")
            .def(py::init<std::shared_ptr<RobotWorldModel>>(), "world_model"_a, "")
            .def("step", &Controller::step,
                 R"doc(
Takes the decision of the robot's next action

You can access the sensors and effectors of the robot by reading and modifying its :attr:`~pyroborobo.Controller.world_model`.

Called at each time step of the simulation.

Examples
---------
>>> def step(self):
...     distance = self.world_model.camera_pixel_distance
...     if np.all(distance < 0.5):
...         # Nothing around, we can go forward without turning
...         self.world_model.translation = 2
...         self.world_model.rotation = 0
...     else:
...         # There are obstacles, we go around in circles slowly
...         self.world_model.translation = 0.5
...         self.world_model.rotation = 15

)doc")
            .def("reset", &Controller::reset, "call at the initialisation of roborobo")
            .def("set_translation", &Controller::setTranslation, "trans_speed"_a,
                 "Set the robot translation speed between [-1, 1]")
            .def("set_rotation", &Controller::setRotation, "rotation_speed"_a,
                 "Set the robot rotation speed between [-1, 1]")
            .def_property_readonly("nb_sensors", [](Controller &self)
            { return self.getWorldModel()->_cameraSensorsNb; }, "int: Number of sensors of the robot")
            .def("get_distance_at", &Controller::getDistanceAt, "sensor_id"_a,
                 R"doc(float: The distance to the object in range [0, 1], if 1, nothing is in sight)doc")
            .def("get_all_distances", [](Controller &self) -> py::array
                 { return as_pyarray(self.getAllDistances()); },
                 R"doc(Return a numpy array of all distances.)doc")
            .def("get_robot_id_at", &Controller::getRobotIdAt, "sensor_id"_a,
                 "Robot: Get the robot instance seen by the sensor ``sensor_id``")
            .def("get_all_robot_ids", [] (Controller& self) {
                return as_pyarray(self.getAllRobotIds());
            })
            .def("get_robot_controller_at", &Controller::getRobotControllerAt, "sensor_id"_a,
                 "Controller: Get the robot's controller seen by the sensor ``sensor_id``", py::return_value_policy::reference)
            .def("get_all_robot_controllers", &Controller::getAllRobotControllers)
            .def("get_robot_instance_at", &Controller::getObjectInstanceAt)
            .def("get_all_robot_instances", &Controller::getAllRobotInstances)
            .def("get_robot_relative_orientation_at", &Controller::getRobotRelativeOrientationAt, "sensor_id"_a,
                 "float: Get robot's relative orientation compared to self seen by the sensor ``sensor_id``")
            .def("get_all_robot_relative_orientations", [] (Controller& self)
            {
                return as_pyarray(self.getAllRobotRelativeOrientations());
            })
            .def("get_object_at", &Controller::getObjectAt, "sensor_id"_a,
                 "Int: Id of the object if seen else -1 for the sensor ``sensor_id``")
            .def("get_all_objects", [] (Controller& self) -> py::array { return as_pyarray(self.getAllObjects()); })
            .def("get_object_instance_at", &Controller::getObjectInstanceAt, "sensor_id"_a,
                 py::return_value_policy::automatic_reference)
            .def("get_all_object_instances", &Controller::getAllObjectInstances)
            .def("get_wall_at", &Controller::getWallAt, "sensor_id"_a,
                 "Bool: Tell if it's a wall seen by the sensor ``sensor_id``")
            .def("get_all_walls", [] (Controller& self) -> py::array { return as_pyarray(self.getAllWalls()); })
            .def("get_ground_sensor_values",
                 [](Controller &self)
                 {
                     return py::make_tuple(self.getRedGroundDetector(),
                                           self.getGreenGroundDetector(), self.getBlueGroundDetector());
                 },
                 R"doc(
tuple[float, float, float]: (red, green, blue) from the ground sensor of the robot between [0,1]
)doc")
            .def("set_color", [](Controller& self, int r, int g, int b) {
                self.getWorldModel()->setRobotLED_colorValues(r, g, b);
            }, R"doc(Set the color of the robot (r,g,b).
:param: r int [0, 255]
:param: g int [0, 255]
:param: b int [0, 255]

)doc")
            .def("get_closest_landmark_dist", &Controller::getClosestLandmarkDistance, "float: The distance to the closest landmark")
            .def("get_closest_landmark_orientation", &Controller::getClosestLandmarkOrientation, "float: The orientation to the closest landmark")
            .def_property_readonly("translation", &Controller::getActualTranslation,
                                   "float: the robot's actual translation speed between [-1, 1]")
            .def_property_readonly("rotation", &Controller::getActualRotation,
                                   "float: the robot's actual rotation speed between [-1, 1]")
            .def_property_readonly("absolute_position", [](Controller &self) -> std::tuple<double, double>
                                   {
                                       auto pos = self.getPosition();
                                       return {pos.x, pos.y};
                                   },
                                   "Tuple[int, int]: Robot's absolute position")
            .def_property_readonly("absolute_orientation", &Controller::getCompass, "Float: Absolute orientation of the robot")
            .def_property_readonly("world_model", &Controller::getWorldModel,
                                   R"doc(
pyroborobo.WorldModel: The robot's world_model
)doc")
            .def_property_readonly("robot", [] (Controller& self) { return gWorld->getRobot(self.getId()); })
            .def_property_readonly("id", &Controller::getId, R"doc(
int: Robot unique ID.
)doc");
}