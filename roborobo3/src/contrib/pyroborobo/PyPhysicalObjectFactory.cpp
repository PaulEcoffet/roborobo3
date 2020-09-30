//
// Created by Paul Ecoffet on 25/09/2020.
//

#include <string>
#include <core/RoboroboMain/roborobo.h>
#include "contrib/pyroborobo/PyPhysicalObjectFactory.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

py::dict PyPhysicalObjectFactory::objectConstructionDict; /* Might raise an exception that cannot be caught. */
std::vector<py::object> PyPhysicalObjectFactory::objectList;

PhysicalObject *PyPhysicalObjectFactory::makeObject(const std::string &type, int id)
{
    PhysicalObject *obj = nullptr;

    if (objectConstructionDict.contains(type))
    {
        py::dict data = PyPhysicalObjectFactory::getObjectData(id);
        py::object pyobj = objectConstructionDict[py::str(type)](id, data);
        obj = pyobj.cast<PhysicalObject *>();
        objectList.emplace_back(pyobj); /* Save the object ref to avoid unexpected gc from python */
    }
    return obj;
}

void PyPhysicalObjectFactory::updateObjectConstructionDict(const py::dict &constructionDict)
{
    PyPhysicalObjectFactory::objectConstructionDict.attr("update")(constructionDict);
}

py::dict PyPhysicalObjectFactory::getObjectData(int id)
{
    py::dict data;
    std::string test = std::string("physicalObjects[") + std::to_string(id) + "].";
    for (const auto &keyval : gProperties)
    {
        const std::string &key = keyval.first;
        const std::string &val = keyval.second;
        if (boost::algorithm::starts_with(key, test))
        {
            std::string innerkey = key.substr(test.size());
            std::string lower_val = boost::to_lower_copy(val);
            bool success = true;
            if (lower_val == "true" || lower_val == "false")
            {
                data[py::str(innerkey)] = lower_val == "true";
            }
            else
            {
                size_t pos = 0;
                long integer_value = 0;
                try
                {
                    integer_value = std::stol(val, &pos);
                }
                catch (std::invalid_argument &e)
                {
                    success = false;
                }
                catch (std::out_of_range &e)
                {
                    success = false;
                }
                if (pos < val.size())
                {
                    success = false;
                }
                if (success)
                {
                    data[py::str(innerkey)] = integer_value;
                }
            }
            if (!success) /* If the conversion to an int has failed */
            {
                success = true;
                size_t pos = 0;
                double double_value = 0;
                try
                {
                    double_value = std::stod(val, &pos);
                }
                catch (std::invalid_argument &e)
                {
                    success = false;
                }
                catch (std::out_of_range &e)
                {
                    success = false;
                }
                if (pos < val.size())
                {
                    success = false;
                }
                if (success)
                {
                    data[py::str(innerkey)] = double_value;
                }
            }
            if (!success)
            {
                data[py::str(innerkey)] = val; /* insert value as string */
            }
        }
    }
    return data;
}
