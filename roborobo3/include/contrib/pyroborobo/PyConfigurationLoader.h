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
#include <Dummy/include/DummyWorldObserver.h>
#include <Dummy/include/DummyAgentObserver.h>
#include <Dummy/include/DummyController.h>

#include <utility>

namespace py = pybind11;

class PyConfigurationLoader : public ConfigurationLoader
{
public:
    PyConfigurationLoader(ConfigurationLoader *configurationLoader, py::object worldObserverClass,
                          py::object agentControllerClass,
                          py::object worldModelClass, py::object agentObserverClass,
                          py::object objectClass)
            :
            fallbackconf(configurationLoader),
            worldObserverClass(std::move(worldObserverClass)),
            agentControllerClass(std::move(agentControllerClass)),
            worldModelClass(std::move(worldModelClass)),
            agentObserverClass(std::move(agentObserverClass)),
            objectClass(std::move(objectClass)),
            allocated()
    {
    }

    ~PyConfigurationLoader() override
    {
        delete fallbackconf;
    }

    WorldObserver *make_WorldObserver(World *w) override
    {
        if (worldObserverClass.is(py::none()))
        {
            return fallbackconf->make_WorldObserver(w);
        }
        else if (py::isinstance<py::str>(worldObserverClass) && worldObserverClass.cast<std::string>() == "dummy")
        {
            return new DummyWorldObserver(w);
        }
        else
        {
            py::object py_worldobserver = worldObserverClass(w);
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
        else if (py::isinstance<py::str>(worldModelClass) && worldModelClass.cast<std::string>() == "dummy")
        {
            return new RobotWorldModel();
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
        else if (py::isinstance<py::str>(agentObserverClass) && agentObserverClass.cast<std::string>() == "dummy")
        {
            return new DummyAgentObserver(wm);
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
        else if (py::isinstance<py::str>(agentControllerClass) && agentControllerClass.cast<std::string>() == "dummy")
        {
            return new DummyController(wm);
        }
        else
        {
            py::object py_controller = agentControllerClass(wm);
            allocated.push_back(py_controller);
            auto *c_controller = py_controller.cast<Controller *>();
            return c_controller;
        }
    }

    PhysicalObject *make_CustomObject(int id) override
    {
        if (agentControllerClass.is(py::none()))
        {
            return fallbackconf->make_CustomObject(id);
        }
        else
        {
            py::object py_object = objectClass(id);
            allocated.push_back(py_object);
            auto *c_object = py_object.cast<PhysicalObject *>();
            return c_object;
        }
    }

private:

    ConfigurationLoader *fallbackconf;
    py::object worldObserverClass;
    py::object agentControllerClass;
    py::object worldModelClass;
    py::object agentObserverClass;
    py::object objectClass;
    std::vector<py::object> allocated; // keep references to the allocated object by the conf loader
};


#endif //ROBOROBO3_PYCONFIGURATIONLOADER_H
