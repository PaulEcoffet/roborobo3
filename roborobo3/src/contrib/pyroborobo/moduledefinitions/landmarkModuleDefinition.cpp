//
// Created by Paul Ecoffet on 18/02/2021.
//

#include <contrib/pyroborobo/ModuleDefinitions/landmarkModuleDefinition.h>
#include <pybind11/pybind11.h>
#include <World/LandmarkObject.h>
#include <pyroborobo/landmarkTrampoline.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addLandmarkBindings(pybind11::module &m)
{
    py::class_<LandmarkObject, LandmarkTrampoline, std::shared_ptr<LandmarkObject> > (m, "Landmark")
    .def(py::init<>(), "Create a Landmark")
    .def("step", &LandmarkObject::step, "Called at each time step")
    .def("set_coordinates", &LandmarkObject::setCoordinates, "x"_a, "y"_a, "Set the landmark at the coordinate (x,y)")
    .def("get_coordinates", [] (LandmarkObject &self) -> std::tuple<int, int> {
        auto pos = self.getCoordinates();
        return {pos.x, pos.y};
        }, "return the (x,y) coordinates of the landmark.")
    .def("hide", [] (LandmarkObject& self) {self.hide(); self._visible = false;})
    .def("show", [] (LandmarkObject& self) {self.show(); self._visible = true;})
    .def_property("radius", &LandmarkObject::getRadius, &LandmarkObject::setRadius)
    .def_readonly("visible", &LandmarkObject::_visible);
}

