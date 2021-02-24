//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyObjectsModuleDefinition.h"
#include <pybind11/pybind11.h>
#include <core/World/SquareObject.h>
#include <contrib/pyroborobo/SquareObjectTrampoline.h>
#include <core/World/CircleObject.h>
#include <contrib/pyroborobo/CircleObjectTrampoline.h>
#include <contrib/pyroborobo/PhysicalObjectTrampoline.h>
#include <World/MovableObject.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addPyObjectsBindings(py::module &m)
{
    py::class_<PhysicalObject, PhysicalObjectTrampoline, std::shared_ptr<PhysicalObject >>(m, "PhysicalObject",
            R"doc(
Abstract Base Class for all roborobo physical objects.

A Physical object is a element in the roborobo simulator with a shape, it can move, disappear, and impact other
objects or robots. Objects have several callbacks that are called by roborobo.

Objects, once created, always exist in the roborobo simulator. Robots and other objects can interact with them if the
objects are *registered*. When an object is registered, that means that the simulator takes its physics into account.

Being registered is different from being visible. An object can be made visible (to the human) using the
:meth:`show` method, or invisible using the :meth:`hide`. If an object is visible and unregistered, it will be visible
to the humans but robot will *not* see them or collide with them. If an object is invisible and registered, the robots
will see them and collide with them but the object will not be rendered on the window or screenshot.

You can force roborobo to display all registered elements using the X-ray mode. You can activate the x-ray mode by
pressing :kbd:`x` on your keyboard when roborobo is rendered in a GUI.

The :meth:`step` method is called at each time step before the robots' step method. The callback :meth:`is_touched`
is called when a robot has the
object in sight, :meth:`is_walked` is called when a robot walk on the "soft" surface of the object, and the method
:meth:`is_pushed` is called when a robot collide with the object.

PhysicalObject cannot be instantiated by itself, only its subclasses :class:`SquareObject` and
:class:`CircleObject` and their subclasses can be created.

Read :doc:`/tuto/objects` for a tutorial on how to use Objects.

)doc")
            .def(py::init<int>(), "id"_a, py::return_value_policy::reference)
            .def("can_register", &PhysicalObject::canRegister, R"doc(
Can the object be register at its position

Returns
-------
bool: can object register at its actual position
)doc")
            .def("register", &PhysicalObject::registerObject, "register the object")
            .def("unregister", &PhysicalObject::unregisterObject, "unregister the object")
            .def("hide", &PhysicalObject::hide, "hide the object from the screen (collision is still active)")
            .def_property_readonly("id", &PhysicalObject::getId, "the id of the object")
            .def("step", &PhysicalObject::step, "Call at each timestep")
            .def("relocate", (void (PhysicalObject::*)()) &PhysicalObject::relocate,
                 "find a random location for the object")
            .def("relocate", (bool (PhysicalObject::*)(double, double)) &PhysicalObject::relocate, "x"_a, "y"_a,
                 "relocate at the (x,y) coordinates")
            .def("is_touched", &PhysicalObject::isTouched, "Triggered when the object is touched")
            .def("is_walked", &PhysicalObject::isWalked, "Triggered when the object is walked on")
            .def("is_pushed", &PhysicalObject::isPushed, "Triggered when the object is pushed")
            .def("set_color", &PhysicalObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 "Set the color (r,g,b) of the object")
            .def("set_coordinates", &PhysicalObject::setCoordinates, "x"_a, "y"_a,
                 "set the coordinates without checking.")
            .def_property_readonly("position", [] (PhysicalObject& self) -> std::tuple<double, double>
                                   {
                                       return {self.getXCenterPixel(), self.getYCenterPixel()};
                                   },
                                   "Return the position of the object"
            );

    py::class_<SquareObject, SquareObjectTrampoline<>, PhysicalObject, std::shared_ptr
            <SquareObject >> (m, "SquareObject", R"doc(
SquareObject(id: int, data: dict) -> SquareObject

Physical object with an axis-aligned rectangular shape

Read :doc:`/tuto/objects` for a tutorial on how to use Objects.

Parameters
----------
id: int
    The id of the object
data: dict (optional)
    The data for the object read from the properties file

See also
--------
PhysicalObject: Reference for physical objects

)doc")
            .def(py::init_alias<int>(), "id"_a,
                 py::return_value_policy::reference,
                 "")
            .def(py::init_alias<int, const py::dict &>(),
                 "id"_a, "data"_a,
                 py::return_value_policy::reference,
                 "")
            .def("can_register",
                 &SquareObject::canRegister,
                 R"(
Can the object be registered at its current location

Returns
-------
bool: Can the object be registered at its current location
)"
            )
            .def("register",
                 &SquareObject::registerObject,
                 "Register the object (activate collision)")
            .def("unregister",
                 &SquareObject::unregisterObject, "Unregister the object (deactivate collision)")
            .def("hide",
                 [](SquareObjectTrampoline<> &self) { self.trueHide(); },
                 "hide the object from the screen (collision is still active)")
            .def("show",
                 [](SquareObjectTrampoline<> &self) { self.trueShow(); },
                 "show the object from the screen (collision is still inactive)")
            .def_property_readonly("id",
                                   &SquareObject::getId, "int: the id of the object")
            .def("step", &SquareObject::step, "Call at each timestep")
            .def("relocate", (void (SquareObject::*)()) &SquareObject::relocate, "Find a new position for the object")
            .def("relocate", (bool (SquareObject::*)(double, double)) &SquareObject::relocate, "x"_a, "y"_a,
                 R"(
Place the object at the (x,y) coordinates. Return if succeed or not

Parameters
----------
x: int
    The x coordinate
y: int
    The y coordinate

Returns
-------
bool: Has the object been relocated

)")
            .def("is_touched", &SquareObject::isTouched, "id"_a, R"(
Callback when the object is touched

Parameters
----------
id: int
    The id of the robot that has touched the object
)")
            .def("is_walked", &SquareObject::isWalked, "id"_a, R"(
Callback when the object is walked on

Parameters
----------
id: int
    The id of the robot that has walked on the object
)")
            .def("is_pushed", &SquareObject::isPushed, "id"_a, "force"_a, R"(
Callback when the object is pushed

Parameters
----------
id: int
    The id of the robot that has pushed the object
speed: tuple(double, double)
    The speed at which the robot pushed the object
)")
            .def("set_color", &SquareObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 "set the (r,g,b) color for the object")
            .def("set_coordinates", &SquareObject::setCoordinates, "x"_a, "y"_a,
                 "force the object to be at the (x,y) coordinates without checking")
            .def_property_readonly("id", &SquareObject::getId, "int: the id of the object")
            .def_readwrite("solid_height", &SquareObjectPublicist::_solid_h,
                           "int: the height of the solid part of the object in pixel")
            .def_readwrite("solid_width", &SquareObjectPublicist::_solid_w,
                           "int: the width of the solid part of the object in pixel")
            .def_readwrite("soft_height", &SquareObjectPublicist::_soft_h,
                           "int: the height of the soft part of the object (can be walked on) in pixel")
            .def_readwrite("soft_width", &SquareObjectPublicist::_soft_w,
                           "int: the width of the soft part of the object (can be walked on) in pixel");

    py::class_<CircleObject, CircleObjectTrampoline<>, PhysicalObject, std::shared_ptr
            <CircleObject >> (m, "CircleObject",
                    R"doc(
CircleObject(id: int, data: dict) -> CircleObject

Physical object with a circular shape

Read :doc:`/tuto/objects` for a tutorial on how to use Objects.

Parameters
----------
id: int
    The id of the object
data: dict (optional)
    The data for the object read from the properties file

See also
--------
PhysicalObject: Reference for physical objects


)doc")
            .def(py::init_alias<int>(), "id"_a,
                 py::return_value_policy::reference,
                 "")
            .def(py::init_alias<int, const py::dict &>(),
                 "id"_a, "data"_a,
                 py::return_value_policy::reference,
                 "")
            .def("can_register",
                 &CircleObject::canRegister,
                 R"doc(
Can the object be registered at its current location

Returns
-------
bool: Can the object be registered at its current location
)doc")
            .def("register",
                 &CircleObject::registerObject,
                 "Register the object (activate collision)")
            .def("unregister",
                 &CircleObject::unregisterObject, "Unregister the object (deactivate collision)")
            .def("hide",
                 [](CircleObjectTrampoline<> &self) { self.trueHide(); },
                 "Hide the object (collision is still active)")
            .def("show",
                 [](CircleObjectTrampoline<> &self) { self.trueShow(); },
                 "Show the object (collision is still inactive)")
            .def_property_readonly("id",
                                   &CircleObject::getId, "int: the id of the object")
            .def("step", &CircleObject::step, "called at each timestep")
            .def("relocate", (void (CircleObject::*)()) &CircleObject::relocate,
                 "Place the object at a random location")
            .def("relocate", (bool (CircleObject::*)(double, double)) &CircleObject::relocate, "x"_a, "y"_a,
                 R"(
Place the object at the (x,y) coordinates. Return False if impossible.

Parameters
----------
x: int
    The x coordinate
y: int
    The y coordinate

Returns
-------
bool: Has the robot been relocated
)"
            )
            .def("is_touched", &CircleObject::isTouched, "id"_a, R"(
Callback when the object is touched

Parameters
----------
id: int
    The id of the robot that has touched the object

)")
            .def("is_walked", &CircleObject::isWalked, "id"_a, R"(
Callback when the object is walked on

Parameters
----------
id: int
    The id of the robot that has walked on the object

)")
            .def("is_pushed", &CircleObject::isPushed, "id"_a, "speed"_a, R"(
Callback when the object is pushed

Parameters
----------
id: int
    The id of the robot that has pushed the object
speed: tuple(double, double)
    The speed at which the robot pushed the object
)")
            .def("set_color", &CircleObject::setDisplayColor, "red"_a, "blue"_a, "green"_a,
                 R"(
Set the color (r,g,b) of the object

Parameters
----------
red: int
    The red component of the color in [0, 255]
blue: int
    The blue component of the color in [0, 255]
green: int
    The green component of the color in [0, 255]
)")
            .def("set_coordinates", &CircleObject::setCoordinates, "x"_a, "y"_a,
                 R"(
Place the object at the (x,y) coordinate without checking

Parameters
----------
x: int
    The x coordinate
y: int
    The y coordinate
)")
            .def_property_readonly("id", &CircleObject::getId, "int: the id of the object")
            .def_readwrite("radius", &CircleObjectPublicist::_radius, "int: The radius of the hard part of the circle")
            .def_readwrite("footprint_radius", &CircleObjectPublicist::_footprintRadius,
                           "int: The radius of the footprint of the object");
    py::class_<MovableObject, CircleObject,
               PyCircleObject<MovableObject>, std::shared_ptr<MovableObject>>(m, "MovableObject")
        .def(py::init<int>(), "id"_a, py::return_value_policy::reference);
}