/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "SingleGenome/include/SingleGenomeAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "SingleGenome/include/SingleGenomeController.h"
#include <cmath>
#include "SingleGenome/include/SingleGenomeWorldObserver.h"
#include <string>

SingleGenomeAgentObserver::SingleGenomeAgentObserver( RobotWorldModel *wm )
{
    _wm = (RobotWorldModel*)wm;
    
}

SingleGenomeAgentObserver::~SingleGenomeAgentObserver()
{
    // nothing to do.
}

void SingleGenomeAgentObserver::reset()
{
    // nothing to do.
}

void SingleGenomeAgentObserver::step()
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
void SingleGenomeAgentObserver::logStats()
{
    SingleGenomeWorldObserver *wobs = static_cast<SingleGenomeWorldObserver *>(gWorld->getWorldObserver());
    SingleGenomeController *ctl = static_cast<SingleGenomeController *>(gWorld->getRobot(_wm->getId())->getController());
    LogManager* coopLogManager = wobs->getCoopLogManager();
    std::stringstream coopStats;
    coopStats << std::setprecision(4);
    // There's one log file per different genome so we don't need to log it
    //fakeRob fakeCoop    Rep Iter    ID  nbRob   Coop
    coopStats << wobs->getNbFakeRobots() << "\t";
    coopStats << wobs->getFakeCoop() << "\t";
    coopStats << wobs->getReplica() << "\t";
    coopStats << wobs->getGenerationItCount() << "\t";
    coopStats << _wm->getId() << "\t";
    coopStats << ctl->getNbRobots() << "\t";
    coopStats << ctl->getCooperationLevel() << "\n";
    coopLogManager->write(coopStats.str());
}

