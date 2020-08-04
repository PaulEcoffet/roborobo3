#include <Lion/include/LionAnalysisOpportunity.h>
#include <Lion/include/LionOpportunity.h>
#include <Negociate/include/NegociateOpportunity.h>
#include "World/PhysicalObjectFactory.h"
#include "World/RoundObject.h"
#include "World/EnergyItem.h"
#include "World/GateObject.h"
#include "World/SwitchObject.h"
#include "World/MovingObject.h"
#include "World/MovableObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"

int PhysicalObjectFactory::_nextId = 0;

void PhysicalObjectFactory::makeObject( int type )
{
    int id = PhysicalObjectFactory::getNextId();

    std::string s = "physicalObject[";
    std::stringstream out;
    out << id;

    s += out.str();
    s += "].type";
    if ( gProperties.hasProperty( s ) )
    {
        convertFromString<int>(type, gProperties.getProperty( s ), std::dec);
    }
    else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] PhysicalObjectFactory: object #" << id << ", type is missing. Assume type "<< gPhysicalObjectDefaultType << "." << std::endl;
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
            gPhysicalObjects.push_back( new EnergyItem(id) );
            break;
        case 2:
            if ( gVerbose )
                std::cout << "[INFO] Gate Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back( new GateObject(id) );
            break;
        case 3:
            if ( gVerbose )
                std::cout << "[INFO] Switch Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back( new SwitchObject(id) );
            break;
        case 4:
            if ( gVerbose )
                std::cout << "[INFO] Movable Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back( new MovableObject(id) );
            break;
        case 5:
            if ( gVerbose )
                std::cout << "[INFO] Moving Object created (type = " << type << ").\n";
            gPhysicalObjects.push_back( new MovingObject(id) );
            break;
        case 6:
            if (gVerbose)
                std::cout << "[INFO] Moving Object 2 max created (type = " << type << ").\n";
            exit(1);
            break;
        case 7:
            if (gVerbose)
                std::cout << "[INFO] Coop Opportunity 2 Fixed created (type = 7).\n";
            exit(1);
            break;
        case 8:
            if (gVerbose)
                std::cout << "[INFO] Partner Choice Opportunity created (type = 8).\n";
            exit(1);
            break;
        case 9:
            if (gVerbose)
                std::cout << "[INFO] Partner Control Opportunity created (type = 9).\n";
            exit(1);
            break;
        case 10:
            if (gVerbose)
                std::cout << "[INFO] CoopFixed2Analysis Opportunity created (type = 10).\n";
            exit(1);
            break;
        case 11:
            if (gVerbose)
                std::cout << "[INFO] CorrectRepartition Opportunity created (type = 11).\n";
            exit(1);
            break;
        case 12:
            if (gVerbose)
                std::cout << "[INFO] Debug Coll obj created (type = 12).\n";
            exit(1);
            break;
        case 13:
            if (gVerbose)
                std::cout << "[INFO] Lion obj created (type = 13).\n";
            gPhysicalObjects.push_back(new LionOpportunity(id));
            break;
        case 14:
            if (gVerbose)
                std::cout << "[INFO] Lion analysis obj created (type = 14).\n";
            gPhysicalObjects.push_back(new LionAnalysisOpportunity(id));
            break;
        case 15:
            if (gVerbose)
                std::cout << "[INFO] Negociate obj created (type = 15).\n";
            gPhysicalObjects.push_back(new NegociateOpportunity(id));
            break;
        case 16:
            if (gVerbose)
                std::cout << "[INFO] Skilled obj created (type = 15).\n";
            exit(1);
            break;
            // case ...: DO NOT FORGET TO UPDATE getNbOfTypes() method.
        default:
            std::cerr << "[CRITICAL] PhysicalObjectFactory: object #" << id << ", type unknown (" << type << ")"
                      << std::endl;
            exit(-1);
    }
}

int PhysicalObjectFactory::getNbOfTypes()
{
    return 16;
}


int PhysicalObjectFactory::getNextId()
{
    int retValue = _nextId;
    _nextId++;
    return retValue;
}
