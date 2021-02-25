#ifndef _PYROBOROBO_DEF_H_
#define _PYROBOROBO_DEF_H_

#include "pyroborobocommon.h"
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

    const std::vector<std::shared_ptr<Controller>> & getControllers();

    const std::vector<std::shared_ptr<WorldModel>> & getWorldModels();

    const std::vector<std::shared_ptr<AgentObserver>> & getAgentObservers();

    std::shared_ptr<WorldObserver> getWorldObserver()
    {
        if (!initialized)
        {
            throw std::runtime_error("World Observer has not been instantiated yet. Have you called roborobo.start()?");
        }
        return wobs;
    }

    const std::vector<std::shared_ptr<Robot>>&getRobots();

    const std::vector<std::shared_ptr<PhysicalObject>>& getObjects();

    static void close();
    void _gatherProjectInstances();


private:

    static Pyroborobo *instance;

    Pyroborobo(const std::string &properties_file,
               py::object &worldObserverClass,
               py::object &agentControllerClass,
               py::object &worldModelClass,
               py::object &agentObserverClass,
               py::dict &objectClassDict,
               const py::dict &options);



    void overrideProperties(const py::dict &dict);

    void initCustomConfigurationLoader(py::object &worldObserverClass,
                                       py::object &agentControllerClass,
                                       py::object &worldModelClass,
                                       py::object &agentObserverClass,
                                       py::object &objectClass);

    Timer fps;
    World *world = nullptr;
    long long currentIt = 0;
    bool initialized = false;
    std::vector<std::shared_ptr<Controller> > controllers;
    std::vector<std::shared_ptr<WorldModel> > worldmodels;
    std::vector<std::shared_ptr<AgentObserver> > agentobservers;
    std::vector<std::shared_ptr<Robot>> robots;
    std::vector<std::shared_ptr<PhysicalObject> > objects;
    std::vector<std::shared_ptr<LandmarkObject> > landmarks;
    std::shared_ptr<WorldObserver> wobs;
    py::object worldObserverClass;
    py::object agentControllerClass;
    py::object worldModelClass;
    py::object agentObserverClass;
    py::dict objectClassDict;
};

#endif