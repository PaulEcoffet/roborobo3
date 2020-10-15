//
// Created by Paul Ecoffet on 25/09/2020.
//

#include <pybind11/pybind11.h>
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotModuleDefinition.h>
#include <core/Agents/Robot.h>
#include <core/RoboroboMain/roborobo.h>
#include <core/Controllers/Controller.h>
#include <core/Observers/AgentObserver.h>
#include <core/WorldModels/RobotWorldModel.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyRobotBinding(py::module &m)
{
    py::class_<Robot, std::shared_ptr<Robot>>(m, "Robot")
            .def("set_position", [](Robot &self, int x, int y) {
                self.unregisterRobot();
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            }, "x"_a, "y"_a, "set the robot at the position (x, y)")
            .def("find_random_location", [](Robot &self) {
                int x, y;
                self.unregisterRobot();
                std::tie(x, y) = self.findRandomLocation(gLocationFinderMaxNbOfTrials);
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            }, "Place the robot at a random location")
            .def_property_readonly("controller", &Robot::getController, "PyController: The robot's controller")
            .def_property_readonly("observer", &Robot::getObserver, "PyAgentObserver: The robot's agent observer")
            .def_property_readonly("world_model", &Robot::getWorldModel, "RobotWorldModel: The robot's world model");
}