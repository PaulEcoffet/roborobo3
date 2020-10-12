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
    py::class_<PhysicalObject, PyPhysicalObjectTrampoline, std::shared_ptr<PhysicalObject >>(m, "PhysicalObject")
            .def(py::init<int>(), "id"_a, py::return_value_policy::reference)
            .def("can_register", &PhysicalObject::canRegister, R"doc(
Can the object be register at its position

Returns
-------

A boolean saying if the object can register at its actual position
)doc")
            .def("register", &PhysicalObject::registerObject, "register the object")
            .def("unregister", &PhysicalObject::unregisterObject, "unregister the object")
            .def("hide", &PhysicalObject::hide, "hide the object from the screen (collision is still active)")
            .def_property_readonly("id", &PhysicalObject::getId, "the id of the object")
            .def("step", &PhysicalObject::step, "Call at each timestep")
            .def("relocate", (void (PhysicalObject::*)()) &PhysicalObject::relocate,
                 "find a random location for the object")
            .def("relocate", (bool (PhysicalObject::*)(int, int)) &PhysicalObject::relocate, "x"_a, "y"_a,
                 "relocate at the (x,y) coordinates")
            .def("is_touched", &PhysicalObject::isTouched, "Triggered when the object is touched")
            .def("is_walked", &PhysicalObject::isWalked, "Triggered when the object is walked on")
            .def("is_pushed", &PhysicalObject::isPushed, "Triggered when the object is pushed")
            .def("set_color", &PhysicalObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 "Set the color (r,g,b) of the object")
            .def("set_coordinates", &PhysicalObject::setCoordinates, "x"_a, "y"_a,
                 "set the coordinates without checking.");

    py::class_ < SquareObject, PySquareObjectTrampoline<>, PhysicalObject, std::shared_ptr
                                                                           < SquareObject >> (m, "SquareObject")
                                                                                   .def(py::init_alias<int>(), "id"_a,
                                                                                        py::return_value_policy::reference,
                                                                                        "")
            .def(py::init_alias<int, const py::dict &>(),
                 "id"_a, "data"_a,
                 py::return_value_policy::reference,
                 "")
            .def("can_register",
                 &SquareObject::canRegister,
                 "Can the object be registered at its current location")
            .def("register",
                 &SquareObject::registerObject,
                 "Register the object (activate collision)")
            .def("unregister",
                 &SquareObject::unregisterObject, "Unregister the object (deactivate collision)")
            .def("hide",
                 [](PySquareObjectTrampoline<> &self) { self.trueHide(); },
                 "hide the object from the screen (collision is still active)")
            .def("show",
                 [](PySquareObjectTrampoline<> &self) { self.trueShow(); },
                 "show the object from the screen (collision is still inactive)")
            .def_property_readonly("id",
                                   &SquareObject::getId, "the id of the object")
            .def("step", &SquareObject::step, "Call at each timestep")
            .def("relocate", (void (SquareObject::*)()) &SquareObject::relocate, "Find a new position for the object")
            .def("relocate", (bool (SquareObject::*)(int, int)) &SquareObject::relocate, "x"_a, "y"_a,
                 "Place the object at the (x,y) coordinates. Return if succeed or not")
            .def("is_touched", &SquareObject::isTouched, "id"_a, "Callback when the object is touched")
            .def("is_walked", &SquareObject::isWalked, "id"_a, "Callback when the object is walked on")
            .def("is_pushed", &SquareObject::isPushed, "id"_a, "force"_a, "Callback when the object is pushed")
            .def("set_color", &SquareObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 "set the (r,g,b) color for the object")
            .def("set_coordinates", &SquareObject::setCoordinates, "x"_a, "y"_a,
                 "force the object to be at the (x,y) coordinates without checking")
            .def_property_readonly("id", &SquareObject::getId, "the id of the object")
            .def_readwrite("solid_height", &SquareObjectPublicist::_solid_h,
                           "the height of the solid part of the object")
            .def_readwrite("solid_width", &SquareObjectPublicist::_solid_w, "the width of the solid part of the object")
            .def_readwrite("soft_height", &SquareObjectPublicist::_soft_h,
                           "the height of the soft part of the object (can be walked on)")
            .def_readwrite("soft_width", &SquareObjectPublicist::_soft_w,
                           "the width of the soft part of the object (can be walked on)");

    py::class_ < CircleObject, PyCircleObjectTrampoline<>, PhysicalObject, std::shared_ptr
                                                                           < CircleObject >> (m, "CircleObject")
                                                                                   .def(py::init_alias<int>(), "id"_a,
                                                                                        py::return_value_policy::reference,
                                                                                        "")
            .def(py::init_alias<int, const py::dict &>(),
                 "id"_a, "data"_a,
                 py::return_value_policy::reference,
                 "")
            .def("can_register",
                 &CircleObject::canRegister,
                 "Can the object be registered at its current location")
            .def("register",
                 &CircleObject::registerObject,
                 "Register the object (activate collision)")
            .def("unregister",
                 &CircleObject::unregisterObject, "Unregister the object (deactivate collision)")
            .def("hide",
                 [](PyCircleObjectTrampoline<> &self) { self.trueHide(); },
                 "Hide the object (collision is still active)")
            .def("show",
                 [](PyCircleObjectTrampoline<> &self) { self.trueShow(); },
                 "Show the object (collision is still inactive)")
            .def_property_readonly("id",
                                   &CircleObject::getId, "the id of the object")
            .def("step", &CircleObject::step, "called at each timestep")
            .def("relocate", (void (CircleObject::*)()) &CircleObject::relocate,
                 "Place the object at a random location")
            .def("relocate", (bool (CircleObject::*)(int, int)) &CircleObject::relocate, "x"_a, "y"_a,
                 "Place the object at the (x,y) coordinates. Return False if impossible")
            .def("is_touched", &CircleObject::isTouched, "id"_a, "Callback when the object is touched")
            .def("is_walked", &CircleObject::isWalked, "id"_a, "Callback when the object is walked on")
            .def("is_pushed", &CircleObject::isPushed, "id"_a, "force"_a, "Callback when the object is pushed")
            .def("set_color", &CircleObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 "Set the color (r,g,b) of the object")
            .def("set_coordinates", &CircleObject::setCoordinates, "x"_a, "y"_a,
                 "Place the object at the (x,y) coordinate without checking")
            .def_property_readonly("id", &CircleObject::getId, "the id of the object")
            .def_readwrite("radius", &CircleObjectPublicist::_radius, "The radius of the hard part of the circle")
            .def_readwrite("footprint_radius", &CircleObjectPublicist::_footprintRadius,
                           "The radius of the footprint of the object");
}