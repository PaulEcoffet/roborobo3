//
// Created by Paul Ecoffet on 25/09/2020.
//

#include "contrib/pyroborobo/PyPhysicalObjectFactory.h"

py::dict PyPhysicalObjectFactory::objectConstructionDict; /* Might raise an exception that cannot be caught. */
std::vector<py::object> PyPhysicalObjectFactory::objectList;

PhysicalObject *PyPhysicalObjectFactory::makeObject(const std::string &type, int id)
{
    PhysicalObject *obj = nullptr;

    if (objectConstructionDict.contains(type))
    {
        py::object pyobj = objectConstructionDict[py::str(type)](id);
        obj = pyobj.cast<PhysicalObject *>();
        objectList.emplace_back(pyobj); /* Save the object ref to avoid unexpected gc from python */
    }
    return obj;
}

void PyPhysicalObjectFactory::updateObjectConstructionDict(const py::dict &constructionDict)
{
    PyPhysicalObjectFactory::objectConstructionDict.attr("update")(constructionDict);
}
