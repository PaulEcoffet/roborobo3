//
// Created by pecoffet on 05/06/2020.
//

#include <pybind11/pybind11.h>


#include <csignal>
#include <core/Config/GlobalConfigurationLoader.h>
#include <core/Agents/InspectorAgent.h>
#include <core/Utilities/Timer.h>
#include <RoboroboMain/roborobo.h>

#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "Controllers/Controller.h"
#include "contrib/pyroborobo/PyControllerTrampoline.h"
#include "contrib/pyroborobo/PyWorldModel.h"
#include "World/World.h"
#include "Config/GlobalConfigurationLoader.h"
#include "PyConfigurationLoader.h"
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

static void quit(int signal)
{
    closeRoborobo();
    exit(0);
}

class Pyroborobo
{
public:
    Pyroborobo(const std::string properties_file,
               py::object worldObserverClass,
               py::object agentControllerClass,
               py::object worldModelClass,
               py::object agentObserverClass,
               py::dict options)
            :
            currentIt(0)
    {
        int argc = 0;
        char *argv[] = {};
        loadProperties(properties_file);
        this->overrideProperties(options);
        this->initCustomConfigurationLoader(worldObserverClass, agentControllerClass,
                                            worldModelClass, agentObserverClass);
    }

    void start()
    {
        currentIt = 0;


        signal(SIGINT, quit);
        signal(SIGTERM, quit);

        /* Taken from init Roborobo */
        gCamera.x = 0;
        gCamera.y = 0;
        gCamera.w = gScreenWidth;
        gCamera.h = gScreenHeight;

        if (!initSDL(SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE))
        {
            py::print("[CRITICAL] cannot initialize SDL: ", SDL_GetError());
            exit(-2);
        }

        // * Initialize log file(s)

        initLogging();


        // * Initialize Random seed -- loaded, or initialized, in loadProperties(...)

        engine.seed(gRandomSeed);

        //srand(gRandomSeed); // fixed seed - useful to reproduce results (ie. deterministic sequence of random values)
        //gLogFile << "# random seed             : " << gRandomSeed << std::endl;

        world = new World();
        gWorld = world;

        // * run

        world->initWorld();


        if (!gBatchMode)
        {
            initMonitor(true);
        } // add speed monitoring and inspector agent

        gatherProjectInstances();


        initialized = true;
    }

    bool update(size_t n_step=1)
    {
        return runRoborobo(n_step);
    }

    std::vector<Controller*> getControllers()
    {
        return controllers;
    }

    std::vector<RobotWorldModel*> getWorldModels()
    {
        return worldmodels;
    }

    std::vector<AgentObserver*> getAgentObservers()
    {
        return agentobservers;
    }

    WorldObserver* getWorldObserver()
    {
        return wobs;
    }

    void close()
    {
        closeRoborobo();
    }

private:

    void gatherProjectInstances()
    {
        size_t nbRob = world->getNbOfRobots();

        controllers.clear();
        controllers.reserve(nbRob);
        worldmodels.clear();
        worldmodels.reserve(nbRob);
        agentobservers.clear();
        agentobservers.reserve(nbRob);

        wobs = world->getWorldObserver();

        for (size_t i = 0; i < nbRob; i++)
        {
            auto *rob = world->getRobot(i);
            worldmodels.emplace_back(rob->getWorldModel());
            agentobservers.emplace_back(rob->getObserver());
            controllers.emplace_back(rob->getController());
        }
    }

    void overrideProperties(const py::dict &dict)
    {
        for (auto elem : dict)
        {
            gProperties.setProperty(elem.first.cast<std::string>(), elem.second.cast<std::string>());
        }
    }

    void initCustomConfigurationLoader(py::object &worldObserverClass,
                                       py::object &agentControllerClass,
                                       py::object &worldModelClass,
                                       py::object &agentObserverClass)
    {
        gConfigurationLoader = new PyConfigurationLoader(gConfigurationLoader, worldObserverClass, agentControllerClass,
                                                         worldModelClass, agentObserverClass);
    }

    InspectorAgent *inspectorAgent = nullptr;
    Timer fps;
    World *world = nullptr;
    int timetag = -1;
    long long currentIt;
    bool initialized;
    std::vector<Controller*> controllers;
    std::vector<RobotWorldModel*> worldmodels;
    std::vector<AgentObserver*> agentobservers;
    WorldObserver* wobs;
};

PYBIND11_MODULE(pyroborobo, m)
{
    py::class_<Pyroborobo>(m, "Pyroborobo")
            .def(py::init<std::string, py::object, py::object, py::object, py::object, py::dict>(),
                 "properties_file"_a,
                 "world_observer_class"_a,
                 "controller_class"_a,
                 "world_model_class"_a,
                 "agent_observer_class"_a,
                 "override_conf_dict"_a)
            .def("start", &Pyroborobo::start)
            .def("update", &Pyroborobo::update, "nb_updates"_a)
            .def("close", &Pyroborobo::close)
            .def("getControllers", &Pyroborobo::getControllers)
            .def("getWorldModels", &Pyroborobo::getWorldModels)
            .def("getAgentObservers", &Pyroborobo::getAgentObservers)
            .def("getWorldObserver", &Pyroborobo::getWorldObserver, py::return_value_policy::reference);
    py::class_<Controller, PyControllerTrampoline>(m, "PyController")
            .def(py::init<>())
            .def(py::init<RobotWorldModel *>(), "world_model"_a)
            .def("step", &Controller::step)
            .def("reset", &Controller::reset)
            .def("getWorldModel", &Controller::getWorldModel);
    py::class_<RobotWorldModel, PyWorldModel>(m, "PyWorldModel")
            .def(py::init<>())
            .def("getCameraSensorsDist", [](PyWorldModel &own)
            { return own.getCameraSensorsDist(); })
            .def("getGroundSensorsValue", [](PyWorldModel &own)
            { return own.getRobotGroundSensors(); })
            .def("getCameraObjectIds", [](PyWorldModel &own)
            { return own.getCameraObjectIds(); })
            .def_property("alive", &RobotWorldModel::isAlive, &RobotWorldModel::setAlive)
            .def_readwrite("translation", &RobotWorldModel::_desiredTranslationalValue)
            .def_readwrite("rotation", &RobotWorldModel::_desiredRotationalVelocity)
            .def_readwrite("fitness", &RobotWorldModel::_fitnessValue);
    py::class_<WorldObserver>(m, "PyWorldObserver")
            .def(py::init<World*>())
            .def("stepPre", &WorldObserver::stepPre)
            .def("stepPost", &WorldObserver::stepPost);
    py::class_<World>(m, "World");
}
