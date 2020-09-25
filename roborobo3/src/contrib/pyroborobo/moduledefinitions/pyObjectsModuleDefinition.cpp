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
            .def("", SquareObject::,)
}