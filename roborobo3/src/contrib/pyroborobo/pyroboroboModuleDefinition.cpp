#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pyroborobo/pyroborobo.h>
#include <core/Controllers/Controller.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include <core/Agents/Robot.h>
#include <contrib/pyroborobo/PyWorldModel.h>
#include <contrib/pyroborobo/PyControllerTrampoline.h>
#include <core/RoboroboMain/roborobo.h>
#include <contrib/pyroborobo/PyCircleObjectTrampoline.h>
#include <contrib/pyroborobo/PySquareObjectTrampoline.h>
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotWorldModelModuleDefinition.h>
#include <contrib/pyroborobo/ModuleDefinitions/pyRobotModuleDefinition.h>
#include <contrib/pyroborobo/ModuleDefinitions/pyObjectsModuleDefinition.h>
#include "contrib/pyroborobo/ModuleDefinitions/pyWorldObserverModuleDefinition.h"
#include "contrib/pyroborobo/ModuleDefinitions/pyControllerModuleDefinition.h"
#include "contrib/pyroborobo/ModuleDefinitions/pyRoboroboModuleDefinition.h"

namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(_pyroborobo, m)
{
    py::options op;
    op.disable_function_signatures();
    addPyRoboroboBinding(m);
    addPyControllerBinding(m);
    addPyRobotWorldModelBinding(m);
    addPyWorldObserverBinding(m);
    py::class_<World> world_def(m, "World");
    addPyRobotBinding(m);
    addPyObjectsBindings(m);


}
