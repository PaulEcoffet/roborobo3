/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */


#include "CoopFixed2/include/CoopFixed2AgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include <cmath>
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include <string>

CoopFixed2AgentObserver::CoopFixed2AgentObserver( RobotWorldModel *wm ) : AgentObserver(wm)
{
    _wm = (RobotWorldModel*)wm;
}

CoopFixed2AgentObserver::~CoopFixed2AgentObserver()
{
    // nothing to do.
}

void CoopFixed2AgentObserver::reset()
{
    // nothing to do.
}

void CoopFixed2AgentObserver::step()
{
    // See whether we're close to an object
    
    int targetIndex = _wm->getGroundSensorValue();
    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
    }
    
}

void CoopFixed2AgentObserver::logStats()
{
    CoopFixed2WorldObserver *wobs = static_cast<CoopFixed2WorldObserver *>(gWorld->getWorldObserver());
    CoopFixed2Controller *ctl = static_cast<CoopFixed2Controller *>(gWorld->getRobot(_wm->getId())->getController());
    std::cout << wobs->getGenerationCount() << "\t";
    std::cout << gWorld->getIterations() << "\t";
    std::cout << _wm->getId() << "\t";
    std::cout << ctl->getNbRobots() << "\t";
    std::cout << ctl->getCooperationLevel() << "\n";
}
