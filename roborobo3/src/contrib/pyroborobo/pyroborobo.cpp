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


namespace py = pybind11;

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
        loadProperties(properties_file, argc, argv);
        this->overrideProperties(options);
        this->initCustomConfigurationLoader(worldObserverClass, agentControllerClass,
                                            worldModelClass, agentObserverClass);
    }

    void start()
    {
        py::print("Roborobo is started !");
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

        py::print("And SDL is initialized\ninitializing logging");
        // * Initialize log file(s)

        initLogging();

        py::print("Logging initialized; seeding engine");

        // * Initialize Random seed -- loaded, or initialized, in loadProperties(...)

        engine.seed(gRandomSeed);

        //srand(gRandomSeed); // fixed seed - useful to reproduce results (ie. deterministic sequence of random values)
        //gLogFile << "# random seed             : " << gRandomSeed << std::endl;

        py::print("creating the world");
        world = new World();
        gWorld = world;

        // * run

        py::print("about to initialize the world");
        world->initWorld();

        py::print("World is initialized");

        if (!gBatchMode)
        {
            initMonitor(true);
        } // add speed monitoring and inspector agent
    }

    bool update(size_t n_step=1)
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
                //Cap the frame rate
                if (fps.get_ticks() < 1000 / gFramesPerSecond)
                {
                    SDL_Delay((1000 / gFramesPerSecond) - fps.get_ticks());
                }

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

    void close()
    {
        closeRoborobo();
    }

private:

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
};

PYBIND11_MODULE(pyroborobo, m)
{
    py::class_<Pyroborobo>(m, "Pyroborobo")
            .def(py::init<std::string, py::object, py::object, py::object, py::object, py::dict>())
            .def("start", &Pyroborobo::start)
            .def("update", &Pyroborobo::update)
            .def("close", &Pyroborobo::close);
    py::class_<Controller, PyControllerTrampoline>(m, "PyController")
            .def(py::init<>())
            .def(py::init<RobotWorldModel *>())
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
            .def_readwrite("speed", &RobotWorldModel::_desiredTranslationalValue)
            .def_readwrite("rotspeed", &RobotWorldModel::_desiredRotationalVelocity)
            .def_readwrite("fitness", &RobotWorldModel::_fitnessValue);
}
