//
// Created by pecoffet on 05/06/2020.
//

#include "contrib/pyroborobo/pyroborobo.h"
#include <pybind11/pybind11.h>

#include <csignal>

#include "RoboroboMain/roborobo.h"
#include "RoboroboMain/main.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "contrib/pyroborobo/PyControllerTrampoline.h"
#include "contrib/pyroborobo/PyWorldModel.h"
#include "PyWorldModelTrampoline.h"


namespace py = pybind11;


class Pyroborobo
{
public:
    Pyroborobo(const py::object &worldObserverClass,
               const py::object &agentControllerClass,
               const py::object &worldModelClass,
               const py::object &agentObserverClass)
    {
        py::object instance = worldObserverClass();
    }

    void start()
    {
        signal(SIGINT, quit);
        signal(SIGTERM, quit);

    }

private:

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
