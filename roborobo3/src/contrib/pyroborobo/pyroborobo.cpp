//
// Created by pecoffet on 05/06/2020.
//



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
#include "PyConfigurationLoader.h"
#include <pybind11/pybind11.h>

#include <pybind11/stl.h>
#include <contrib/pyroborobo/pyroborobo.h>


namespace py = pybind11;
using namespace pybind11::literals;

Pyroborobo::Pyroborobo(const std::string &properties_file,
                       py::object &worldObserverClass,
                       py::object &agentControllerClass,
                       py::object &worldModelClass,
                       py::object &agentObserverClass,
                       py::object &objectClass,
                       const py::dict &options)
        :
        currentIt(0)
{

    loadProperties(properties_file);
    this->overrideProperties(options);
    this->initCustomConfigurationLoader(worldObserverClass, agentControllerClass,
                                        worldModelClass, agentObserverClass, objectClass);
}

void Pyroborobo::start()
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

bool Pyroborobo::update(size_t n_step)
{
    bool quit = false;
    for (size_t i = 0; i < n_step && !quit; i++)
    {
        if (gBatchMode)
        {
            gWorld->updateWorld();
            if (gWorld->getIterations() % 10000 == 0)
                if (gVerbose)
                    std::cout << ".";

            // monitor trajectories (if needed)
            if (gTrajectoryMonitor)
                updateTrajectoriesMonitor();
        }
        else
        {
            const Uint8 *keyboardStates = SDL_GetKeyboardState(nullptr);
            quit = checkEvent() | handleKeyEvent(keyboardStates);

            //Start the frame timer
            fps.start();
            if (!gPauseMode)
            {
                if (gUserCommandMode && !gInspectorMode)
                    gWorld->updateWorld(keyboardStates);
                else
                    gWorld->updateWorld();
            }
            //Update the screen
            updateDisplay();

            // monitor trajectories (if needed)
            if (gTrajectoryMonitor)
                updateTrajectoriesMonitor();

            updateMonitor(keyboardStates);
        }

        currentIt++;
        if (gWorld->getNbOfRobots() <= 0)
        {
            quit = true;
        }
    }
    return quit;
}

std::vector<Controller *> Pyroborobo::getControllers()
{
    if (!initialized)
    {
        throw std::runtime_error("Controllers have not been instanciated yet. Have you called roborobo.start()?");
    }
    return controllers;
}

std::vector<RobotWorldModel *> Pyroborobo::getWorldModels()
{
    if (!initialized)
    {
        throw std::runtime_error("World models have not been instantiated yet. Have you called roborobo.start()?");
    }
    return worldmodels;
}

std::vector<AgentObserver *> Pyroborobo::getAgentObservers()
{
    if (!initialized)
    {
        throw std::runtime_error(
                "Agent Observers have not been instantiated yet. Have you called roborobo.start()?");
    }
    return agentobservers;
}

std::vector<Robot *> Pyroborobo::getRobots()
{
    if (!initialized)
    {
        throw std::runtime_error("Robots have not been instantiated yet. Have you called roborobo.start()?");
    }
    return robots;
}

void Pyroborobo::close()
{
    closeRoborobo();
}

void Pyroborobo::gatherProjectInstances()
{
    size_t nbRob = world->getNbOfRobots();

    controllers.clear();
    controllers.reserve(nbRob);
    worldmodels.clear();
    worldmodels.reserve(nbRob);
    agentobservers.clear();
    agentobservers.reserve(nbRob);
    robots.clear();
    robots.reserve(nbRob);

    wobs = world->getWorldObserver();

    for (size_t i = 0; i < nbRob; i++)
    {
        auto *rob = world->getRobot(i);
        robots.emplace_back(rob);
        worldmodels.emplace_back(rob->getWorldModel());
        agentobservers.emplace_back(rob->getObserver());
        controllers.emplace_back(rob->getController());
    }
}

void Pyroborobo::overrideProperties(const py::dict &dict)
{
    for (auto elem : dict)
    {
        gProperties.setProperty(py::str(elem.first), py::str(elem.second));
    }
}

void Pyroborobo::initCustomConfigurationLoader(py::object &worldObserverClass, py::object &agentControllerClass,
                                               py::object &worldModelClass, py::object &agentObserverClass,
                                               py::object &objectClass)
{
    gConfigurationLoader = new PyConfigurationLoader(gConfigurationLoader, worldObserverClass, agentControllerClass,
                                                     worldModelClass, agentObserverClass, objectClass);
}

int Pyroborobo::addObjectToEnv(PhysicalObject *obj)
{
    int id = PhysicalObjectFactory::getNextId();
    obj->setId(id);
    gPhysicalObjects.emplace_back(obj);
    return id;
}