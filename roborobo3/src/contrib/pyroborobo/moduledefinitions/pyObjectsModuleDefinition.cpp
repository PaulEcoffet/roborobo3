//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyObjectsModuleDefinition.h"

#include <pybind11/pybind11.h>
#include <core/World/SquareObject.h>
#include <contrib/pyroborobo/PySquareObjectTrampoline.h>
#include <core/World/CircleObject.h>
#include <contrib/pyroborobo/PyCircleObjectTrampoline.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyObjectsBindings(py::module &m)
{
    py::class_<SquareObject, PySquareObjectTrampoline<> >(m, "SquareObject")
            .def(py::init_alias<int>(), "id"_a, py::return_value_policy::reference)
            .def(py::init_alias<int, const py::dict &>(), "id"_a, "data"_a, py::return_value_policy::reference)
            .def("can_register", &SquareObject::canRegister)
            .def("register", &SquareObject::registerObject)
            .def("unregister", &SquareObject::unregisterObject)
            .def("hide", &SquareObject::hide)
            .def_property_readonly("id", &SquareObject::getId)
            .def("step", &SquareObject::step)
            .def("relocate", (void (SquareObject::*)()) &SquareObject::relocate)
            .def("relocate", (bool (SquareObject::*)(int, int)) &SquareObject::relocate, "x"_a, "y"_a)
            .def("is_touched", &SquareObject::isTouched)
            .def("is_walked", &SquareObject::isWalked)
            .def("is_pushed", &SquareObject::isPushed)
            .def("set_color", &SquareObject::setDisplayColor)
            .def("set_coordinates", &SquareObject::setCoordinates, "x"_a, "y"_a)
            .def_readwrite("solid_height", &SquareObjectPublicist::_solid_h)
            .def_readwrite("solid_width", &SquareObjectPublicist::_solid_w)
            .def_readwrite("soft_height", &SquareObjectPublicist::_soft_h)
            .def_readwrite("soft_width", &SquareObjectPublicist::_soft_w);

    py::class_<CircleObject, PyCircleObjectTrampoline<> >(m, "CircleObject")
            .def(py::init_alias<int>(), "id"_a, py::return_value_policy::reference)
            .def(py::init_alias<int, const py::dict &>(), "id"_a, "data"_a, py::return_value_policy::reference)
            .def("can_register", &CircleObject::canRegister)
            .def("register", &CircleObject::registerObject)
            .def("unregister", &CircleObject::unregisterObject)
            .def("hide", &CircleObject::hide)
            .def_property_readonly("id", &CircleObject::getId)
            .def("step", &CircleObject::step)
            .def("relocate", (void (CircleObject::*)()) &CircleObject::relocate)
            .def("relocate", (bool (CircleObject::*)(int, int)) &CircleObject::relocate, "x"_a, "y"_a)
            .def("is_touched", &CircleObject::isTouched)
            .def("is_walked", &CircleObject::isWalked)
            .def("is_pushed", &CircleObject::isPushed);
}