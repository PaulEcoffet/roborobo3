/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 */


#include <DebugColl/include/DebugCollObj.h>
#include "DebugColl/include/DebugCollWorldObserver.h"
#include "World/World.h"
#include "RoboroboMain/main.h"
#include "WorldModels/RobotWorldModel.h"


DebugCollWorldObserver::DebugCollWorldObserver( World *__world ) : WorldObserver( __world ), log(gLogDirectoryname + "/debuglog2.txt")
{
	_world = __world;
}

DebugCollWorldObserver::~DebugCollWorldObserver()
{
	// nothing to do.
}

void DebugCollWorldObserver::reset()
{
    auto* rob = _world->getRobot(0);
    rob->getWorldModel()->_agentAbsoluteOrientation = 0;
    log << "it,rot,pushed\n";
}

void DebugCollWorldObserver::stepPre()
{
    auto* rob = _world->getRobot(0);
    auto* phys = gPhysicalObjects[0];
    rob->setCoord(phys->getXCenterPixel(), phys->getYCenterPixel());
    rob->setCoordReal(phys->getXCenterPixel(), phys->getYCenterPixel());
}

void DebugCollWorldObserver::stepPost()
{
    bool pushed = dynamic_cast<DebugCollObj*>(gPhysicalObjects[0])->_pushed;
    auto *wm = _world->getRobot(0)->getWorldModel();
    log << _world->getIterations() << "," << wm->_agentAbsoluteOrientation << "," << pushed << "\n";
    if (!pushed)
    {
        std::cout << _world->getIterations() << "\n";
        std::cout << "nocol\n" << wm->_agentAbsoluteOrientation << "\n";
    }
}
