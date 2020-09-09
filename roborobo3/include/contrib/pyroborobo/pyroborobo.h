#ifndef _PYROBOROBO_H_
#define _PYROBOROBO_H_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Agents/Robot.h>
#include <World/World.h>
#include <Observers/WorldObserver.h>
#include <Observers/AgentObserver.h>
#include <WorldModels/RobotWorldModel.h>
#include <Controllers/Controller.h>
#include <Utilities/Timer.h>
#include <Agents/InspectorAgent.h>

namespace py = pybind11;
using namespace pybind11::literals;

class Pyroborobo
{
public:
    Pyroborobo(const std::string properties_file,
               py::object worldObserverClass,
               py::object agentControllerClass,
               py::object worldModelClass,
               py::object agentObserverClass,
               py::dict options);

    void start();

    bool update(size_t n_step=1);

    std::vector<Controller*> getControllers();

    std::vector<RobotWorldModel*> getWorldModels();

    std::vector<AgentObserver*> getAgentObservers();

    WorldObserver *getWorldObserver()
    {
        if (wobs == nullptr || !initialized)
        {
            throw std::runtime_error("World Observer has not been instantiated yet. Have you called roborobo.start()?");
        }
        return wobs;
    }

    std::vector<Robot *> getRobots();

    void close();

private:

    void gatherProjectInstances();

    void overrideProperties(const py::dict &dict);

    void initCustomConfigurationLoader(py::object &worldObserverClass,
                                       py::object &agentControllerClass,
                                       py::object &worldModelClass,
                                       py::object &agentObserverClass);

    InspectorAgent *inspectorAgent = nullptr;
    Timer fps;
    World *world = nullptr;
    int timetag = -1;
    long long currentIt = 0;
    bool initialized = false;
    std::vector<Controller *> controllers;
    std::vector<RobotWorldModel *> worldmodels;
    std::vector<AgentObserver *> agentobservers;
    std::vector<Robot *> robots;
    WorldObserver *wobs = nullptr;
};

#endif