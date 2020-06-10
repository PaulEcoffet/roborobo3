//
// Created by pecoffet on 05/06/2020.
//

#include <pybind11/pybind11.h>

#include <csignal>
#include <core/Config/GlobalConfigurationLoader.h>

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
    Pyroborobo(const std::string &properties_file,
               const py::object &worldObserverClass,
               const py::object &agentControllerClass,
               const py::object &worldModelClass,
               const py::object &agentObserverClass,
               const py::dict &options = py::dict())
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
        gLogFile << "# random seed             : " << gRandomSeed << std::endl;

        gWorld = new World();

        // * run
        gWorld->initWorld();

        if (!gBatchMode)
        {
            initMonitor(true);
        } // add speed monitoring and inspector agent
    }

    void update()
    {

    }

private:

    void overrideProperties(const py::dict &dict)
    {

    }

    void initCustomConfigurationLoader(const py::object &worldObserverClass,
                                       const py::object &agentControllerClass,
                                       const py::object &worldModelClass,
                                       const py::object &agentObserverClass)
    {
        gConfigurationLoader = new PyConfigurationLoader(gConfigurationLoader, worldObserverClass, agentControllerClass,
                                                         worldModelClass, agentObserverClass);
    }
};

PYBIND11_MODULE(pyroborobo, m)
{
    py::class_<Pyroborobo>(m, "Pyroborobo")
            .def(py::init<py::object, py::object, py::object, py::object>())
            .def("start", &Pyroborobo::start);
    /*.def("start", &Pyroborobo::start)
    .def("setActions", &Pyroborobo::setActions);
    .def("getObservations", &Pyroborobo::getObservations)
    .def("getProperties", &Pyroborobo::getProperties)
    .def("setProperty", &Pyroborobo::setProperty)*/
    py::class_<Controller, PyControllerTrampoline>(m, "PyControllerTrampoline")
            .def(py::init<>())
            .def(py::init<RobotWorldModel *>())
            .def("step", &Controller::step)
            .def("reset", &Controller::reset)
            .def("getWorldModel", &Controller::getWorldModel);
    py::class_<PyWorldModel>(m, "PyWorldModel")
            .def(py::init<>())
            .def("getCameraSensorsDist", &PyWorldModel::getCameraSensorsDist)
            .def("getGroundSensorsValue", &PyWorldModel::getGroundSensorValue)
            .def("getCameraObjectIds", &PyWorldModel::getCameraObjectIds)
            .def_property("alive", &PyWorldModel::isAlive, &PyWorldModel::setAlive)
            .def_readwrite("speed", &PyWorldModel::_desiredTranslationalValue)
            .def_readwrite("rotspeed", &PyWorldModel::_desiredRotationalVelocity)
            .def_readwrite("fitness", &PyWorldModel::_fitnessValue);
}
