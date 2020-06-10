//
// Created by pecoffet on 10/06/2020.
//

#ifndef ROBOROBO3_PYCONFIGURATIONLOADER_H
#define ROBOROBO3_PYCONFIGURATIONLOADER_H


#include <ext/Config/ConfigurationLoader.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

class PyConfigurationLoader : public ConfigurationLoader
{
public:
    PyConfigurationLoader(ConfigurationLoader *gConfigurationLoader, py::object &worldObserverClass,
                          py::object &agentControllerClass,
                          py::object &worldModelClass, py::object &agentObserverClass)
            :
            fallbackconf(gConfigurationLoader),
            worldObserverClass(worldObserverClass),
            agentControllerClass(agentControllerClass),
            worldModelClass(worldModelClass),
            agentObserverClass(agentObserverClass)
    {}

    WorldObserver *make_WorldObserver(World *wm) override
    {
        if (worldObserverClass.is(py::none()))
        {
            return fallbackconf->make_WorldObserver(wm);
        }
        else
        {
            py::object py_worldobserver = worldObserverClass();
            auto *c_worldobserver = py_worldobserver.cast<WorldObserver *>();
            return c_worldobserver;
        }
    }

    RobotWorldModel *make_RobotWorldModel() override
    {

    }

    AgentObserver *make_AgentObserver(RobotWorldModel *wm) override
    {

    }

    Controller *make_Controller(RobotWorldModel *wm) override
    {

    }

private:

    ConfigurationLoader *fallbackconf;
    py::object &worldObserverClass;
    py::object &agentControllerClass;
    py::object &worldModelClass;
    py::object &agentObserverClass;
};


#endif //ROBOROBO3_PYCONFIGURATIONLOADER_H
