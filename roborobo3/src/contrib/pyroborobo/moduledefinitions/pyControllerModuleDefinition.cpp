//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyAgentObserverModuleDefinition.h"
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotWorldModelModuleDefinition.h>
#include <contrib/pyroborobo/PySquareObjectTrampoline.h>
#include <contrib/pyroborobo/PyCircleObjectTrampoline.h>
#include <core/RoboroboMain/roborobo.h>
#include <contrib/pyroborobo/PyControllerTrampoline.h>
#include <contrib/pyroborobo/PyWorldModel.h>
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
    pybind11::class_<Controller, PyControllerTrampoline, std::shared_ptr<Controller>>(m, "PyController",
                                                                                      R"doc(
Class to extend a Roborobo Controller in python.

..warning:
    If the __init__ is override, it is absolutely necessary to call the PyController constructor by doing :
    ```
    def __init__(self, world_model):
        PyController.__init__(self, world_model)
    ```

    Not doing so leads to cryptic errors due to the interface between python and c++.

    It is also necessary to override `step` and `reset`. Not doing so leads to cryptic errors.
)doc")
            .def(py::init<std::shared_ptr<RobotWorldModel>>(), "world_model"_a)
            .def("step", &Controller::step,
                 R"doc(
Takes the decision of the robot's next action by reading and modifying its `world_model`.
Called at each time step of the simulation.

Example
-------

```
def step(self):
    distance = self.world_model.get_camera_sensors_dist()
    if all(distance < 0.5):
        # Nothing around, we can go forward without turning #
        self.world_model.translation = 2
        self.world_model.rotation = 0
    else:
        # There are obstacles, we go around in circles slowly #
        self.world_model.translation = 0.5
        self.world_model.rotation = 15
```
)doc")
            .def("reset", &Controller::reset)
            .def_property_readonly("world_model", &Controller::getWorldModel,
                                   R"doc(
The robot's world_model.
)doc")
            .def_property_readonly("id", &Controller::getId);
}