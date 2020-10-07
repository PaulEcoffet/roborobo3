//
// Created by Paul Ecoffet on 25/09/2020.
//

#include <pybind11/pybind11.h>
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotModuleDefinition.h>
#include <core/Agents/Robot.h>
#include <core/RoboroboMain/roborobo.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyRobotBinding(py::module &m)
{
    py::class_<Robot, std::shared_ptr<Robot>>(m, "PyRobot")
            .def("set_position", [](Robot &self, int x, int y)
            {
                self.unregisterRobot();
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            }, "x"_a, "y"_a)
            .def("find_random_location", [](Robot &self)
            {
                int x, y;
                self.unregisterRobot();
                std::tie(x, y) = self.findRandomLocation(gLocationFinderMaxNbOfTrials);
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            });
}