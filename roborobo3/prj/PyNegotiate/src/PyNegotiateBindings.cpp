//
// Created by pecoffet on 05/08/2020.
//

#include <pybind11/pybind11.h>
#include <PyNegotiate/include/PyNegotiateWorldModel.h>

namespace py = pybind11;


PYBIND11_MODULE(negotiate, m)
{
    m.attr("__name__") = "pyroborobo.negotiate";  // Define the right name to make a submodule
    m.doc() = "Negotiate documentation";
    py::module::import("pyroborobo");  // give access to pyroborobo methods

    py::class_<PyNegotiateWorldModel>(m, "NegotiateWorldModel")
            .def("coop", &PyNegotiateWorldModel::_cooperationLevel);
}