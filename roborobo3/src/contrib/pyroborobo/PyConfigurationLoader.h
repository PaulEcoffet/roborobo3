//
// Created by pecoffet on 10/06/2020.
//

#ifndef ROBOROBO3_PYCONFIGURATIONLOADER_H
#define ROBOROBO3_PYCONFIGURATIONLOADER_H


#include <ext/Config/ConfigurationLoader.h>
#include <World/World.h>
#include <WorldModels/RobotWorldModel.h>
#include <Observers/WorldObserver.h>
#include <Observers/AgentObserver.h>
#include <Controllers/Controller.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

class PyConfigurationLoader : public ConfigurationLoader
{
public:
    PyConfigurationLoader(ConfigurationLoader *configurationLoader, const py::object &worldObserverClass,
                          const py::object &agentControllerClass,
                          const py::object &worldModelClass, const py::object &agentObserverClass)
            :
            fallbackconf(configurationLoader),
            worldObserverClass(worldObserverClass),
            agentControllerClass(agentControllerClass),
            worldModelClass(worldModelClass),
            agentObserverClass(agentObserverClass),
            allocated()
    {
    }

    ~PyConfigurationLoader() override
    {
        delete fallbackconf;
    }

    WorldObserver *make_WorldObserver(World *wm) override
    {
        if (worldObserverClass.is(py::none()))
        {
            return fallbackconf->make_WorldObserver(wm);
        }
        else
        {
            py::object py_worldobserver = worldObserverClass();
            allocated.push_back(py_worldobserver);
            auto *c_worldobserver = py_worldobserver.cast<WorldObserver *>();
            return c_worldobserver;
        }
    }

    RobotWorldModel *make_RobotWorldModel() override
    {

        if (worldModelClass.is(py::none()))
        {
            return fallbackconf->make_RobotWorldModel();
        }
        else
        {
            py::object py_worldmodel = worldModelClass();
            allocated.push_back(py_worldmodel);
            auto *c_worldmodel = py_worldmodel.cast<PyWorldModel *>();
            return c_worldmodel;
        }
    }

    AgentObserver *make_AgentObserver(RobotWorldModel *wm) override
    {

        if (agentObserverClass.is(py::none()))
        {
            return fallbackconf->make_AgentObserver(wm);
        }
        else
        {
            py::object py_agentobserver = agentObserverClass(wm);
            allocated.push_back(py_agentobserver);
            auto *c_agentobserver = py_agentobserver.cast<AgentObserver *>();
            return c_agentobserver;
        }
    }

    Controller *make_Controller(RobotWorldModel *wm) override
    {

        if (agentControllerClass.is(py::none()))
        {
            return fallbackconf->make_Controller(wm);
        }
        else
        {
            py::object py_controller = agentControllerClass(wm);
            allocated.push_back(py_controller);
            auto *c_controller = py_controller.cast<PyControllerTrampoline *>();
            return c_controller;
        }
    }

private:

    ConfigurationLoader *fallbackconf;
    py::object worldObserverClass;
    py::object agentControllerClass;
    py::object worldModelClass;
    py::object agentObserverClass;
    std::vector<py::object> allocated; // keep references to the allocated object by the conf loader
};


#endif //ROBOROBO3_PYCONFIGURATIONLOADER_H
