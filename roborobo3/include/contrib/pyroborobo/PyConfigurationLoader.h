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
            worldObserverClass(worldObserverClass),
            agentControllerClass(agentControllerClass),
            worldModelClass(worldModelClass),
            agentObserverClass(agentObserverClass),
            objectClass(objectClass),
            allocated()
    {
    }

    ~PyConfigurationLoader() override
    {
        delete fallbackconf;
    }

    std::shared_ptr<WorldObserver> make_WorldObserver(std::shared_ptr<World> w) override
    {
        if (worldObserverClass.is(py::none()))
        {
            return fallbackconf->make_WorldObserver(w);
        }
        else if (py::isinstance<py::str>(worldObserverClass) && worldObserverClass.cast<std::string>() == "dummy")
        {
            return std::make_shared<DummyWorldObserver>(w);
        }
        else
        {
            py::object py_worldobserver = worldObserverClass(w);
            allocated.push_back(py_worldobserver);
            auto c_worldobserver = py_worldobserver.cast<std::shared_ptr<WorldObserver>>();
            return c_worldobserver;
        }
    }

    std::shared_ptr<RobotWorldModel> make_RobotWorldModel() override
    {

        if (worldModelClass.is(py::none()))
        {
            return fallbackconf->make_RobotWorldModel();
        }
        else if (py::isinstance<py::str>(worldModelClass) && worldModelClass.cast<std::string>() == "dummy")
        {
            return std::make_shared<RobotWorldModel>();
        }
        else
        {
            py::object py_worldmodel = worldModelClass();
            allocated.push_back(py_worldmodel);
            auto c_worldmodel = py_worldmodel.cast<std::shared_ptr<PyRobotWorldModel<>>>();
            return c_worldmodel;
        }
    }

    std::shared_ptr<AgentObserver> make_AgentObserver(std::shared_ptr<RobotWorldModel> wm) override
    {

        if (agentObserverClass.is(py::none()))
        {
            return fallbackconf->make_AgentObserver(wm);
        }
        else if (py::isinstance<py::str>(agentObserverClass) && agentObserverClass.cast<std::string>() == "dummy")
        {
            return std::make_shared<DummyAgentObserver>(wm);
        }
        else
        {
            py::object py_agentobserver = agentObserverClass(wm);
            allocated.push_back(py_agentobserver);
            auto c_agentobserver = py_agentobserver.cast<std::shared_ptr<AgentObserver> >();
            return c_agentobserver;
        }
    }

    std::shared_ptr<Controller> make_Controller(std::shared_ptr<RobotWorldModel> wm) override
    {

        if (agentControllerClass.is(py::none()))
        {
            return fallbackconf->make_Controller(wm);
        }
        else if (py::isinstance<py::str>(agentControllerClass) && agentControllerClass.cast<std::string>() == "dummy")
        {
            return std::make_shared<DummyController>(wm);
        }
        else
        {
            py::object py_controller = agentControllerClass(wm);
            allocated.push_back(py_controller);
            auto c_controller = py_controller.cast<std::shared_ptr<Controller>>();
            return c_controller;
        }
    }

    std::shared_ptr<PhysicalObject> make_CustomObject(int id) override
    {
        return std::shared_ptr<PhysicalObject>(nullptr);
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
