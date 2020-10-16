//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/ModuleDefinitions/pyRoboroboModuleDefinition.h"

#include <pybind11/pybind11.h>
#include <pybind11/attr.h>
#include <contrib/pyroborobo/pyroborobo.h>

namespace py = pybind11;
using namespace pybind11::literals;


void addPyRoboroboBinding(py::module &m)
{
    py::class_<Pyroborobo>(m, "Pyroborobo", R"doc(
        Python interface to the roborobo simulator

)doc")
            .def_static("create", &Pyroborobo::createRoborobo,
                        "properties_file"_a,
                        "world_observer_class"_a = py::none(),
                        "controller_class"_a = py::none(),
                        "world_model_class"_a = py::none(),
                        "agent_observer_class"_a = py::none(),
                        "object_class_dict"_a = py::dict(),
                        "override_conf_dict"_a = py::dict(),
                        R"doc(
Create the singleton Pyroborobo

Parameters
----------
properties_file: str
    Properties file for the roborobo simulator

world_observer_class: Class inherited from PyWorldObserver or str or None
    Class used to instantiate the WorldObserver. It must inherit from :class:`~pyroborobo.PyWorldObserver`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

controller_class: Class inherited from PyController or str or None
    Class used to instantiate the Controllers. It must inherit from :class:`~pyrobobobo.PyController`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

world_model_class: Class inherited from PyWorldModel or str or None
    Class used to instantiate the WorldModels. It must inherit from :class:`~pyroborobo.PyWorldModel`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

agent_observer_class: Class inherited from PyAgentObserver or str or None
    Class used to instantiate the AgentObservers. It must inherit from :class:`~pyroborobo.PyAgentObserver`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

object_class_dict: Dict of string -> class inherited from PhysicalObject
    Dict of classes that are used to instantiate physical object from the pytype argument in conf file.
    The classes must be inherited from :class:`pyroborobo.PyCircleObject`, :class:`pyroborobo.PySquareObject` and their subclasses.

override_conf_dict: dict
    Dictionary which updates the configuration file loaded by the ``properties_file`` parameter.
    Dictionary key/value pairs already present in the configuration file are overwritten. Key/value pairs that do not
    exist in the configuration file are added.

    DOES NOT WORK YET (dunno why)

Returns
-------
pyroborobo.Pyroborobo: The pyroborobo instance
)doc")
            .def_static("get", &Pyroborobo::get, R"doc(
Return the pyroborobo instance

Returns
-------
pyroborobo.Pyroborobo
)doc")
            .def("start", &Pyroborobo::start,
                 R"doc(
Starts the simulator.

Starts the simulator, creates the window if the batch mode is not activated. Once started, it is impossible to
modify the classes used to instantiate the different modules of the simulator or to change its configuration.
)doc")
            .def("update", &Pyroborobo::update, "nb_updates"_a,
                 R"doc(
Performs a simulator evaluation of ``nb_updates`` time steps.

It is possible to call `update` multiple times. If the simulator is paused, then the frame generation has no impact
on the number of updates.

Parameters
----------
self: pyroborobo.Pyroborobo
nb_updates: int
    The number of simulator updates to do.
    If the simulator is paused, then the frame generation has no impact on the number of updates.

Returns
-------
bool
    Has the end of the simulation been requested, either by roborobo itself or by closing the window

Examples
---------

>>> roborobo.update(1000)  # run simulation for 1000 time steps
>>> agents.learn()  # trigger agents' learning algorithms
>>> roborobo.update(1000)  # Continue the simulation for 1000 more time steps

)doc")
            .def("close", &Pyroborobo::close,
                 R"doc(
Exit the simulator nicely

Once the simulator is closed, it cannot be reopen in the same python interpreter.
)doc")
            .def_property_readonly("controllers", &Pyroborobo::getControllers, py::return_value_policy::reference,
                                   R"doc(
:class:`list` of :class:`~pyroborobo.Controller`: The ordered list of all the controllers in the simulation

.. warning::
    :meth:`~pyroborobo.Pyroborobo.start` must have been called before using this property
)doc")
            .def_property_readonly("world_models", &Pyroborobo::getWorldModels, py::return_value_policy::reference,
                                   R"doc(
:class:`list` of :class:`~pyroborobo.WorldModel`: The ordered list of all the world models in the simulation

.. warning::
    :meth:`~pyroborobo.Pyroborobo.start` must have been called before using this property
)doc")
            .def_property_readonly("agent_observers", &Pyroborobo::getAgentObservers,
                                   py::return_value_policy::reference,
                                   R"doc(
:class:`list` of :class:`~pyroborobo.AgentObserver`: The ordered list of all the agent observers in the simulation

.. warning::
    :meth:`~pyroborobo.Pyroborobo.start` must have been called before using this property
)doc")
            .def_property_readonly("world_observer", &Pyroborobo::getWorldObserver, py::return_value_policy::reference,
                                   R"doc(
:obj:`pyroborobo.WorldObserver`: The World Observer of the roborobo environment

.. warning::
    :meth:`~pyroborobo.Pyroborobo.start` must have been called before using this property
)doc")
            .def_property_readonly("robots", &Pyroborobo::getRobots, py::return_value_policy::reference,
                                   R"doc(
:class:`list` of :class:`pyroborobo.Robot`: The ordered list of all the robots in the simulation

.. warning::
    :meth:`~pyroborobo.Pyroborobo.start` must have been called before using this property
)doc")
            .def_property_readonly("objects", &Pyroborobo::getObjects, R"doc(
:class:`list` of :class:`pyroborobo.PhysicalObject`: The ordered list of all the objects in the simulation.
)doc")
            .def_property_readonly("robot_index_offset", []()
            { return gRobotIndexStartOffset; }, R"doc(
int: Index at which the robot ids start

Everything under this offset is a physical objects. Everything above is a robot.
)doc");
}
