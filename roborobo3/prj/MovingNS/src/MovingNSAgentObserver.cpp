/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "MovingNS/include/MovingNSAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "MovingNS/include/MovingNSController.h"
#include <cmath>
#include "MovingNS/include/MovingNSWorldObserver.h"
#include <string>

MovingNSAgentObserver::MovingNSAgentObserver( RobotWorldModel *wm )
{
    _wm = (RobotWorldModel*)wm;
    
}

MovingNSAgentObserver::~MovingNSAgentObserver()
{
    // nothing to do.
}

void MovingNSAgentObserver::reset()
{
    // nothing to do.
}

void MovingNSAgentObserver::step()
{
    // See whether we're close to an object
    
    int targetIndex = _wm->getGroundSensorValue();
    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
    }
    
}