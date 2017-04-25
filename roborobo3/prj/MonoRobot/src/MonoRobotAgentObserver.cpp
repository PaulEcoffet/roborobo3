/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "MonoRobot/include/MonoRobotAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "MonoRobot/include/MonoRobotController.h"
#include <cmath>
#include "MonoRobot/include/MonoRobotWorldObserver.h"
#include <string>

MonoRobotAgentObserver::MonoRobotAgentObserver( RobotWorldModel *wm )
{
    _wm = (RobotWorldModel*)wm;
    
}

MonoRobotAgentObserver::~MonoRobotAgentObserver()
{
    // nothing to do.
}

void MonoRobotAgentObserver::reset()
{
    // nothing to do.
}

void MonoRobotAgentObserver::step()
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
