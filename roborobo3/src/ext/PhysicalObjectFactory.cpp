#include <CoopFixed2/include/CoopFixed2Opportunity.h>
#include <PartnerChoice/include/PartnerChoiceOpportunity.h>
#include <PartnerControl/include/PartnerControlOpportunity.h>
#include "World/PhysicalObjectFactory.h"
#include "World/RoundObject.h"
#include "World/EnergyItem.h"
#include "World/GateObject.h"
#include "World/SwitchObject.h"
#include "World/MovingObject.h"
#include "World/MovableObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxMovingObject2Max.h"

int PhysicalObjectFactory::_nextId = 0;

void PhysicalObjectFactory::makeObject( int type )
{
    int id = PhysicalObjectFactory::getNextId();

    std::string s = "";
    std::stringstream out;
    out << id;

    s = "physicalObject[";
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
            gPhysicalObjects.push_back( new CoopOpportunity2MaxMovingObject2Max(id));
            break;
        case 7:
            if (gVerbose)
                std::cout << "[INFO] Coop Opportunity 2 Fixed created (type = 7).\n";
            gPhysicalObjects.push_back( new CoopFixed2Opportunity(id));
            break;
        case 8:
            if (gVerbose)
                std::cout << "[INFO] Partner Choice Opportunity created (type = 7).\n";
            gPhysicalObjects.push_back( new PartnerChoiceOpportunity(id));
            break;
        case 9:
            if (gVerbose)
                std::cout << "[INFO] Partner Control Opportunity created (type = 7).\n";
            gPhysicalObjects.push_back( new PartnerControlOpportunity(id));
            break;
        // case ...: DO NOT FORGET TO UPDATE getNbOfTypes() method.
        default:
            std::cerr << "[CRITICAL] PhysicalObjectFactory: object #" << id << ", type unknown (" << type << ")" << std::endl;
            exit(-1);
    }
}

int PhysicalObjectFactory::getNbOfTypes()
{
    return 9;
}


int PhysicalObjectFactory::getNextId()
{
    int retValue = _nextId;
    _nextId++;
    return retValue;
}
