//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyObjectsModuleDefinition.h"
#include <pybind11/pybind11.h>
#include <core/World/SquareObject.h>
#include <contrib/pyroborobo/PySquareObjectTrampoline.h>
#include <core/World/CircleObject.h>
#include <contrib/pyroborobo/PyCircleObjectTrampoline.h>
#include <contrib/pyroborobo/PyPhysicalObjectTrampoline.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyObjectsBindings(py::module &m)
{
    py::class_ < PhysicalObject, PyPhysicalObjectTrampoline, std::shared_ptr < PhysicalObject >> (m, "PhysicalObject")
            .def(py::init<int>(), "id"_a, py::return_value_policy::reference)
            .def("can_register", &PhysicalObject::canRegister)
            .def("register", &PhysicalObject::registerObject)
            .def("unregister", &PhysicalObject::unregisterObject)
            .def("hide", &PhysicalObject::hide)
            .def_property_readonly("id", &PhysicalObject::getId)
            .def("step", &PhysicalObject::step)
            .def("relocate", (void (PhysicalObject::*)()) &PhysicalObject::relocate)
            .def("relocate", (bool (PhysicalObject::*)(int, int)) &PhysicalObject::relocate, "x"_a, "y"_a)
            .def("is_touched", &PhysicalObject::isTouched)
            .def("is_walked", &PhysicalObject::isWalked)
            .def("is_pushed", &PhysicalObject::isPushed)
            .def("set_color", &PhysicalObject::setDisplayColor, "red"_a, "blue"_a, "green"_a)
            .def("set_coordinates", &PhysicalObject::setCoordinates, "x"_a, "y"_a);

    py::class_ < SquareObject, PySquareObjectTrampoline<>, PhysicalObject, std::shared_ptr
                                                                           < SquareObject >> (m, "SquareObject")
                                                                                   .def(py::init_alias<int>(), "id"_a,
                                                                                        py::return_value_policy::reference)
                                                                                   .def(py::init_alias<int, const py::dict &>(),
                                                                                        "id"_a, "data"_a,
                                                                                        py::return_value_policy::reference)
                                                                                   .def("can_register",
                                                                                        &SquareObject::canRegister)
                                                                                   .def("register",
                                                                                        &SquareObject::registerObject)
                                                                                   .def("unregister",
                                                                                        &SquareObject::unregisterObject)
                                                                                   .def("hide",
                                                                                        [](PySquareObjectTrampoline<> &self)
                                                                                        { self.trueHide(); })
                                                                                   .def("show",
                                                                                        [](PySquareObjectTrampoline<> &self)
                                                                                        { self.trueShow(); })
                                                                                   .def_property_readonly("id",
                                                                                                          &SquareObject::getId)
            .def("step", &SquareObject::step)
            .def("relocate", (void (SquareObject::*)()) &SquareObject::relocate)
            .def("relocate", (bool (SquareObject::*)(int, int)) &SquareObject::relocate, "x"_a, "y"_a)
            .def("is_touched", &SquareObject::isTouched)
            .def("is_walked", &SquareObject::isWalked)
            .def("is_pushed", &SquareObject::isPushed)
            .def("set_color", &SquareObject::setDisplayColor, "red"_a, "blue"_a, "green"_a)
            .def("set_coordinates", &SquareObject::setCoordinates, "x"_a, "y"_a)
            .def_property_readonly("id", &SquareObject::getId)
            .def_readwrite("solid_height", &SquareObjectPublicist::_solid_h)
            .def_readwrite("solid_width", &SquareObjectPublicist::_solid_w)
            .def_readwrite("soft_height", &SquareObjectPublicist::_soft_h)
            .def_readwrite("soft_width", &SquareObjectPublicist::_soft_w);

    py::class_ < CircleObject, PyCircleObjectTrampoline<>, PhysicalObject, std::shared_ptr
                                                                           < CircleObject >> (m, "CircleObject")
                                                                                   .def(py::init_alias<int>(), "id"_a,
                                                                                        py::return_value_policy::reference)
                                                                                   .def(py::init_alias<int, const py::dict &>(),
                                                                                        "id"_a, "data"_a,
                                                                                        py::return_value_policy::reference)
                                                                                   .def("can_register",
                                                                                        &CircleObject::canRegister)
                                                                                   .def("register",
                                                                                        &CircleObject::registerObject)
                                                                                   .def("unregister",
                                                                                        &CircleObject::unregisterObject)
                                                                                   .def("hide",
                                                                                        [](PyCircleObjectTrampoline<> &self)
                                                                                        { self.trueHide(); })
                                                                                   .def("show",
                                                                                        [](PyCircleObjectTrampoline<> &self)
                                                                                        { self.trueShow(); })
                                                                                   .def_property_readonly("id",
                                                                                                          &CircleObject::getId)
            .def("step", &CircleObject::step)
            .def("relocate", (void (CircleObject::*)()) &CircleObject::relocate)
            .def("relocate", (bool (CircleObject::*)(int, int)) &CircleObject::relocate, "x"_a, "y"_a)
            .def("is_touched", &CircleObject::isTouched)
            .def("is_walked", &CircleObject::isWalked)
            .def("is_pushed", &CircleObject::isPushed)
            .def("set_color", &CircleObject::setDisplayColor, "red"_a, "blue"_a, "green"_a)
            .def("set_coordinates", &CircleObject::setCoordinates, "x"_a, "y"_a)
            .def_property_readonly("id", &CircleObject::getId)
            .def_readwrite("radius", &CircleObjectPublicist::_radius)
            .def_readwrite("footprint_radius", &CircleObjectPublicist::_footprintRadius);
}