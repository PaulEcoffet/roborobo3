//
// Created by Paul Ecoffet on 25/09/2020.
//

#ifndef ROBOROBO3_PYPHYSICALOBJECTFACTORY_H
#define ROBOROBO3_PYPHYSICALOBJECTFACTORY_H


#include <core/World/PhysicalObject.h>
#include <string>
#include <pybind11/pybind11.h>

namespace py = pybind11;

class PyPhysicalObjectFactory
{
private:
    static py::dict objectConstructionDict;
    static std::vector<py::object> objectList;


public:
    PyPhysicalObjectFactory() = delete;

    static PhysicalObject *makeObject(const std::string &type, int id);

    static void updateObjectConstructionDict(const py::dict &constructionDict);

};


#endif //ROBOROBO3_PYPHYSICALOBJECTFACTORY_H
