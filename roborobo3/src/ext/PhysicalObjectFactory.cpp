#include <core/Config/GlobalConfigurationLoader.h>
#include "World/PhysicalObjectFactory.h"
#include "World/RoundObject.h"
#include "World/EnergyItem.h"
#include "World/GateObject.h"
#include "World/SwitchObject.h"
#include "World/MovableObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"

#ifdef PYROBOROBO

#include <contrib/pyroborobo/PyPhysicalObjectFactory.h>

#endif

int PhysicalObjectFactory::_nextId = 0;

void PhysicalObjectFactory::makeObject(int type)
{
    int id = PhysicalObjectFactory::getNextId();

    std::string s;
    std::string pytype;
    std::stringstream out;
    out << id;

    s = "physicalObject[";
    s += out.str();
    pytype = s + "].pytype";
    s += "].type";
    if (gProperties.hasProperty(pytype))
    {
#ifdef PYROBOROBO
        std::string strpytype;
        gProperties.checkAndGetPropertyValue(pytype, &strpytype, false);
        gPhysicalObjects.push_back(PyPhysicalObjectFactory::makeObject(pytype, id));
#else
        throw std::runtime_error("Python is not activated here, please use pyroborobo from a python interpreter"
                                 "to run this conf\n");
#endif

    }
    if (gProperties.hasProperty(s))
    {
        convertFromString<int>(type, gProperties.getProperty(s), std::dec);
    }
    else
    {
        if (gVerbose)
            std::cerr << "[MISSING] PhysicalObjectFactory: object #" << id << ", type is missing. Assume type "
                      << gPhysicalObjectDefaultType << "." << std::endl;
        type = gPhysicalObjectDefaultType;
    }

    switch ( type )
    {
        case 0:
            if ( gVerbose )
                std::cout << "[INFO] Round Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back( new RoundObject(id) );
            break;
        case 1:
            if ( gVerbose )
                std::cout << "[INFO] Energy Item created (type = " << type << ").\n";
            gPhysicalObjects.push_back(new EnergyItem(id) );
            break;
        case 2:
            if ( gVerbose )
                std::cout << "[INFO] Gate Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back(new GateObject(id) );
            break;
        case 3:
            if (gVerbose)
            {
                std::cout << "[INFO] Switch Object created (type = " << type << ").\n";
            }
            gPhysicalObjects.push_back(new SwitchObject(id));
            break;
        case 4:
            if (gVerbose)
            {
                std::cout << "[INFO] Movable Object created (type = " << type << ").\n";
            }
            gPhysicalObjects.push_back(new MovableObject(id));
            break;
        case -1:
            if (gVerbose)
            {
                std::cout << "[INFO] Custom Object created (type = " << type << ").\n";
            }
            gPhysicalObjects.push_back(gConfigurationLoader->make_CustomObject(id));
            // case ...: DO NOT FORGET TO UPDATE getNbOfTypes() method.
        default:
            std::cerr << "[CRITICAL] PhysicalObjectFactory: object #" << id << ", type unknown (" << type << ")"
                      << std::endl;
            exit(-1);
    }
}

int PhysicalObjectFactory::getNbOfTypes()
{
    return 6;
}


int PhysicalObjectFactory::getNextId()
{
    int retValue = _nextId;
    _nextId++;
    return retValue;
}
