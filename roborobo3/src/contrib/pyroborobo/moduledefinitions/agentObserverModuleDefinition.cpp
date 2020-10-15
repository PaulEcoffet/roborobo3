//
// Created by pecoffet on 15/10/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/agentObserverModuleDefinition.h"
#include "contrib/pyroborobo/AgentObserverTrampoline.h"

namespace py = pybind11;
using namespace pybind11::literals;

void addAgentObserverBindings(pybind11::module &m)
{
    py::class_<AgentObserver, AgentObserverTrampoline, std::shared_ptr<AgentObserver> >(m, "AgentObserver")
            .def(py::init<std::shared_ptr<RobotWorldModel>>(), "world_model"_a)
            .def("reset", &AgentObserver::reset)
            .def("step_pre", &AgentObserver::stepPre)
            .def("step_post", &AgentObserver::stepPost);
}