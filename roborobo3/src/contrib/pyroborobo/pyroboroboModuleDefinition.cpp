#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pyroborobo/pyroborobo.h>
#include <core/Controllers/Controller.h>
#include <core/WorldModels/RobotWorldModel.h>
#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include <core/Agents/Robot.h>
#include <contrib/pyroborobo/PyWorldModel.h>
#include <contrib/pyroborobo/PyControllerTrampoline.h>
#include <core/RoboroboMain/roborobo.h>

#include <utility>

namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(pyroborobo, m)
{
    py::class_<Pyroborobo>(m, "Pyroborobo", R"doc(
        Python interface to the roborobo simulator

)doc")
            .def(py::init<std::string, py::object, py::object, py::object, py::object, py::dict>(),
                 "properties_file"_a,
                 "world_observer_class"_a,
                 "controller_class"_a,
                 "world_model_class"_a,
                 "agent_observer_class"_a,
                 "override_conf_dict"_a,
                 R"doc(
Parameters
----------
properties_file: str
    Properties file for the roborobo simulator
world_observer_class: Class inherited from PyWorldObserver or str or None
    Class used to instantiate the WorldObserver. It must inherit from `PyWorldObserver`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

controller_class: Class inherited from PyController or str or None
    Class used to instantiate the Controllers. It must inherit from `PyController`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

world_model_class: Class inherited from PyWorldModel or str or None
    Class used to instantiate the WorldModels. It must inherit from `PyWorldModel`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

agent_observer_class: Class inherited from PyAgentObserver or str or None
    Class used to instantiate the AgentObservers. It must inherit from `PyAgentObservers`.
    It is possible to pass `None`, in which case the C++ class loaded by the loader configuration is used.
    Finally, it is possible to pass the string "dummy", in this case, a minimal class that does not perform any action
    is used.

override_conf_dict: dict
    Dictionary which updates the configuration file loaded by the `properties_file` parameter.
    Dictionary key/value pairs already present in the configuration file are overwritten. Key/value pairs that do not
    exist in the configuration file are added.

    DOES NOT WORK YET
)doc")
            .def("start", &Pyroborobo::start,
                    R"doc(
Starts the simulator, creates the window if the batch mode is not activated. Once started, it is impossible to
modify the classes used to instantiate the different modules of the simulator or to change its configuration.
)doc")
            .def("update", &Pyroborobo::update, "nb_updates"_a,
                 R"doc(
Performs a simulator evaluation of `nb_updates' time steps. It is possible to call `update' multiple times.

Returns
-------

quit: bool
    quit: has the end of the simulation been requested, either by roborobo itself or by closing the window

Example
------------

>>> roborobo.update(1000)  # run simulation for 1000 time steps
>>> agents.learn()  # trigger agents' learning algorithms
>>> roborobo.update(1000)  # Continue the simulation for 1000 more time steps
)doc")
            .def("close", &Pyroborobo::close,
                    R"doc(
Exit the simulator nicely
)doc")
            .def_property_readonly("controllers", &Pyroborobo::getControllers, py::return_value_policy::reference,
                    R"doc(
The List of all the controllers in the Roborobo environment in order

`PyRoborobo.start()` must have been called before using this property
)doc")
            .def_property_readonly("world_models", &Pyroborobo::getWorldModels, py::return_value_policy::reference,
                    R"doc(
The list of all world models of the roborobo environment in order

`PyRoborobo.start()` must have been called before using this property
)doc")
            .def_property_readonly("agent_observers", &Pyroborobo::getAgentObservers, py::return_value_policy::reference,
                    R"doc(
The list of all agent observers of the roborobo environment in order

`PyRoborobo.start()` must have been called before using this property
)doc")
            .def_property_readonly("world_observer", &Pyroborobo::getWorldObserver, py::return_value_policy::reference,
                    R"doc(
The World Observer of the roborobo environment

`PyRoborobo.start()` must have been called before using this property
)doc")
            .def_property_readonly("robots", &Pyroborobo::getRobots, py::return_value_policy::reference,
                    R"doc(
The list of all the robots of the roborobo environment in order

`PyRoborobo.start()` must have been called before using this property
)doc");
    py::class_<Controller, PyControllerTrampoline>(m, "PyController",
                                                   R"doc(
Class to extend a Roborobo Controller in python.

..warning:
    If the __init__ is override, it is absolutely necessary to call the PyController constructor by doing:
    ```
    def __init__(self, world_model):
        PyController.__init__(self, world_model)
    ```

    Not doing so leads to cryptic errors due to the interface between python and c++.

    It is also necessary to override `step` and `reset`. Not doing so leads to cryptic errors.
)doc")
            .def(py::init<RobotWorldModel *>(), "world_model"_a)
            .def("step", &Controller::step,
                    R"doc(
Takes the decision of the robot's next action by reading and modifying its `world_model`.
Called at each time step of the simulation.

Example
-------

```
def step(self):
    distance = self.world_model.get_camera_sensors_dist()
    if all(distance < 0.5):
        # Nothing around, we can go forward without turning #
        self.world_model.translation = 2
        self.world_model.rotation = 0
    else:
        # There are obstacles, we go around in circles slowly #
        self.world_model.translation = 0.5
        self.world_model.rotation = 15
```
)doc")
            .def("reset", &Controller::reset)
            .def_property_readonly("world_model", &Controller::getWorldModel,
                    R"doc(
The robot's world_model.
)doc");
    py::class_<RobotWorldModel, PyWorldModel>(m, "PyWorldModel")
            .def(py::init<>())
            .def("get_camera_sensors_dist", [](PyWorldModel &self) { return self.getCameraSensorsDist(); },
                 R"doc(
Returns a numpy array of the normalized distances of the objects seen by the robot sensors. If the distance is 1,
no object is seen by the sensor. If the distance is 0, the object is completely adjacent to the robot.

Returns
-------
A numpy.array of the objects seen by the robot's sensors.
)doc")
            .def("get_ground_sensor_values", [](PyWorldModel &self) { return self.getRobotGroundSensors(); },
                 R"doc(
Returns a tuple (red, green, blue) from the ground sensor of the robot
)doc")
            .def("get_camera_object_ids", [](PyWorldModel &self) { return self.getCameraObjectIds(); },
                 R"doc(
Returns a numpy array of the IDs of the objects seen by the robot
)doc")
            .def("get_obs", [](PyWorldModel &self) { return self.getObservations(); },
                 R"doc(
Returns a python object containing the obs you need for your python controller.  It can be implemented at your will.
It can return a numpy.array, a dict, a single value or whatever you want.

It is useful when you want to implement a gym environment.
)doc")
            .def("set_actions", [](PyWorldModel &self, py::object actions) {
                     self.setActions(std::move(actions));
                 },
                 R"doc(
Feed the world model with the action the robot intend to do. It can be implemented at your will.
It can receive a numpy.array, a dict, a single value or whatever you want.

It is useful when you want to implement a gym environment.
)doc")
            .def_property("alive", &RobotWorldModel::isAlive, &RobotWorldModel::setAlive)
            .def_readwrite("translation", &RobotWorldModel::_desiredTranslationalValue)
            .def_readwrite("rotation", &RobotWorldModel::_desiredRotationalVelocity)
            .def_readwrite("fitness", &RobotWorldModel::_fitnessValue);
    py::class_<WorldObserver>(m, "PyWorldObserver")
            .def(py::init<World *>(), "World"_a, py::return_value_policy::reference)
            .def("step_pre", &WorldObserver::stepPre)
            .def("step_post", &WorldObserver::stepPost);
    py::class_<World>(m, "World");
    py::class_<Robot>(m, "PyRobot")
            .def("set_position", [](Robot &self, int x, int y) {
                self.unregisterRobot();
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            }, "x"_a, "y"_a)
            .def("find_random_location", [](Robot &self) {
                int x, y;
                self.unregisterRobot();
                std::tie(x, y) = self.findRandomLocation(gLocationFinderMaxNbOfTrials);
                self.setCoord(x, y);
                self.setCoordReal(x, y);
                self.registerRobot();
            });
}
