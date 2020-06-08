//
// Created by pecoffet on 05/06/2020.
//

#include "contrib/pyroborobo/pyroborobo.h"
#include <pybind11/pybind11.h>
#include <csignal>
#include <RoboroboMain/roborobo.h>
#include "Controllers/Controller.h"


namespace py = pybind11;

void quit(int signal)
{
    closeRoborobo();
    exit(0);
}

class PyController : public Controller {
public:
    /* Inherit the constructors */
    using Controller::Controller;

    /* Trampoline (need one for each virtual function) */
    void step() override {
        PYBIND11_OVERLOAD(
                void, /* Return type */
                Controller,      /* Parent class */
                step,          /* Name of function in C++ (must match Python name) */
        );
    }
};


class Pyroborobo
{
public:
    Pyroborobo(py::object worldObserverClass,
            py::object agentControllerClass,
            py::object worldModelClass,
            py::object agentObserverClass)
    {
        py::object instance = worldObserverClass();
    }

    void init()
    {
        signal(SIGINT, quit);
        signal(SIGTERM, quit);

    }

private:

};

PYBIND11_MODULE(pyroborobo, m) {
    py::class_<Pyroborobo>(m, "Pyroborobo")
            .def(py::init<py::object, py::object, py::object, py::object>())
            .def("init", &Pyroborobo::init)
            /*.def("start", &Pyroborobo::start)
            .def("setActions", &Pyroborobo::setActions);
            .def("getObservations", &Pyroborobo::getObservations)
            .def("getProperties", &Pyroborobo::getProperties)
            .def("setProperty", &Pyroborobo::setProperty)*/
            ;
}
