#ifndef _PYROBOROBO_DEF_H_
#define _PYROBOROBO_DEF_H_

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

    static Pyroborobo *createRoborobo(const std::string &properties_file,
                                      py::object &worldObserverClass,
                                      py::object &agentControllerClass,
                                      py::object &worldModelClass,
                                      py::object &agentObserverClass,
                                      py::dict &objectClassDict,
                                      const py::dict &options);

    ~Pyroborobo();

    static Pyroborobo *get();

    void start();

    bool update(size_t n_step = 1);

    int addObjectToEnv(PhysicalObject *);

    const std::vector<Controller *> &getControllers();

    const std::vector<RobotWorldModel *> &getWorldModels();

    const std::vector<AgentObserver *> &getAgentObservers();

    WorldObserver *getWorldObserver()
    {
        if (wobs == nullptr || !initialized)
        {
            throw std::runtime_error("World Observer has not been instantiated yet. Have you called roborobo.start()?");
        }
        return wobs;
    }

    const std::vector<Robot *> &getRobots();

    const std::vector<PhysicalObject *> &getObjects();

    static void close();

private:

    static Pyroborobo *instance;

    Pyroborobo(const std::string &properties_file,
               py::object &worldObserverClass,
               py::object &agentControllerClass,
               py::object &worldModelClass,
               py::object &agentObserverClass,
               py::dict &objectClassDict,
               const py::dict &options);


    void gatherProjectInstances();

    void overrideProperties(const py::dict &dict);

    void initCustomConfigurationLoader(py::object &worldObserverClass,
                                       py::object &agentControllerClass,
                                       py::object &worldModelClass,
                                       py::object &agentObserverClass,
                                       py::object &objectClass);

    InspectorAgent *inspectorAgent = nullptr;
    Timer fps;
    World *world = nullptr;
    long long currentIt = 0;
    bool initialized = false;
    std::vector<Controller *> controllers;
    std::vector<RobotWorldModel *> worldmodels;
    std::vector<AgentObserver *> agentobservers;
    std::vector<Robot *> robots;
    std::vector<PhysicalObject *> objects;
    WorldObserver *wobs = nullptr;
};

#endif