//
// Created by Paul Ecoffet on 25/09/2020.
//

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
#include "contrib/pyroborobo/ModuleDefinitions/pyAgentObserverModuleDefinition.h"

void addPyAgentObserverBinding(py::module &m)
{
    pybind11::class_<WorldObserver, std::shared_ptr<WorldObserver>>(m, "PyWorldObserver")
            .def(py::init<World *>(), "World"_a, py::return_value_policy::reference)
            .def("step_pre", &WorldObserver::stepPre)
            .def("step_post", &WorldObserver::stepPost);
}