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

    static std::shared_ptr<Pyroborobo>createRoborobo(const std::string &properties_file,
                                      py::object &worldObserverClass,
                                      py::object &agentControllerClass,
                                      py::object &worldModelClass,
                                      py::object &agentObserverClass,
                                      py::dict &objectClassDict,
                                      const py::dict &options);

    ~Pyroborobo();

    static std::shared_ptr<Pyroborobo> get();

    void start();

    bool update(size_t n_step = 1);

    std::shared_ptr<PhysicalObject> addObjectToEnv(std::shared_ptr<PhysicalObject>);

    const std::vector<py::object> & getControllers() const;

    const std::vector<py::object> & getWorldModels() const;

    const std::vector<py::object> & getAgentObservers() const;

    py::object getWorldObserver()
    {
        if (!initialized)
        {
            throw std::runtime_error("World Observer has not been instantiated yet. Have you called roborobo.start()?");
        }
        return pywobs;
    }

    const std::vector<py::object>& getRobots() const;

    const std::vector<py::object>& getObjects() const;

    static void close();
    void _gatherProjectInstances();

    Pyroborobo(const std::string &properties_file,
               py::object &worldObserverClass,
               py::object &agentControllerClass,
               py::object &worldModelClass,
               py::object &agentObserverClass,
               py::dict &objectClassDict,
               const py::dict &options);



private:

    static std::shared_ptr<Pyroborobo> instance;




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
    std::vector<py::object > controllers;
    std::vector<py::object > worldmodels;
    std::vector<py::object > agentobservers;
    std::vector<py::object > robots;
    std::vector<py::object > objects;
    std::vector<py::object > landmarks;
    std::shared_ptr<WorldObserver> wobs;
    py::object pywobs;
    py::object worldObserverClass;
    py::object agentControllerClass;
    py::object worldModelClass;
    py::object agentObserverClass;
    py::dict objectClassDict;
};

#endif