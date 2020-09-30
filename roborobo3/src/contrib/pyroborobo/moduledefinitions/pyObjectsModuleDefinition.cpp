//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "pyObjectsModuleDefinition.h"

#include <pybind11/pybind11.h>
#include <core/World/SquareObject.h>
#include <contrib/pyroborobo/PySquareObjectTrampoline.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyObjectsDefinitions(py::module &m)
{
    py::class_<SquareObject, PySquareObjectTrampoline<> >(m, "SquareObject")
            .def(py::init<int>(), "id"_a, py::return_value_policy::reference)
            .def("can_register", &SquareObject::canRegister)
            .def("register", &SquareObject::registerObject)
            .def("unregister", &SquareObject::unregisterObject)
            .def("hide", &SquareObject::hide)
            .def_property_readonly("id", &SquareObject::getId)
            .def("step", &SquareObject::step)
            .def("relocate", &SquareObject::relocate)
            .def("relocate", &SquareObject::relocate, "x"_a, "y"_a)


}