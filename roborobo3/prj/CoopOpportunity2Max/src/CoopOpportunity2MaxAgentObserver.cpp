/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */


#include "CoopOpportunity2Max/include/CoopOpportunity2MaxAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxController.h"
#include <cmath>
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxWorldObserver.h"
#include <string>

CoopOpportunity2MaxAgentObserver::CoopOpportunity2MaxAgentObserver(RobotWorldModel *wm)
{
    _wm = (RobotWorldModel *) wm;

}

CoopOpportunity2MaxAgentObserver::~CoopOpportunity2MaxAgentObserver()
{
    // nothing to do.
}

void CoopOpportunity2MaxAgentObserver::reset()
{
    // nothing to do.
}

void CoopOpportunity2MaxAgentObserver::stepPre()
{
    // See whether we're close to an object

    int targetIndex = _wm->getGroundSensorValue();
    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
    }

}

void CoopOpportunity2MaxAgentObserver::logStats()
{
    CoopOpportunity2MaxWorldObserver *wobs = static_cast<CoopOpportunity2MaxWorldObserver *>(gWorld->getWorldObserver());
    CoopOpportunity2MaxController *ctl = static_cast<CoopOpportunity2MaxController *>(gWorld->getRobot(
            _wm->getId())->getController());
    std::cout << wobs->getGenerationCount() << "\t";
    std::cout << gWorld->getIterations() << "\t";
    std::cout << _wm->getId() << "\t";
    std::cout << ctl->getNbRobots() << "\t";
    std::cout << ctl->getCooperationLevel() << "\n";
}
