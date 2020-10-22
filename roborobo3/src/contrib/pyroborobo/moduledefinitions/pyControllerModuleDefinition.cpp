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
            .def("get_robot_id_at", &Controller::getRobotIdAt, "sensor_id"_a, "Get the robot instance seen by the sensor ``sensor_id``")
            .def("get_robot_controller_at", &Controller::getRobotControllerAt, "sensor_id"_a, "Get the robot's controller seen by the sensor ``sensor_id``")
            .def("get_object_at", &Controller::getObjectAt, "sensor_id"_a, "Get the physical object seen by the sensor ``sensor_id``")
            .def("get_wall_at", &Controller::getWallAt, "sensor_id"_a, "Tell if it's a wall seen by the sensor ``sensor_id``")
            .def_property_readonly("absolute_position", [](Controller& self) -> std::tuple<double, double> {
                auto pos = self.getPosition();
                return {pos.x, pos.y};
                },
                          "Robot's absolute position")
            .def_property_readonly("world_model", &Controller::getWorldModel,
                                   R"doc(
pyroborobo.WorldModel: The robot's world_model
)doc")
            .def_property_readonly("id", &Controller::getId, R"doc(
int: Robot unique ID.
)doc");
}