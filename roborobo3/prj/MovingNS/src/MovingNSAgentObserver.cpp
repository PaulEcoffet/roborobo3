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
    // * update fitness (if needed)
    if ( _wm->isAlive() && _wm->getPushed() )
    {
        MovingNSController *ctl = dynamic_cast<MovingNSController *>(gWorld->getRobot(_wm->getId())->getController());
        ctl->updateFitness(ctl->getFitness()+1);
    }
    _wm->setPushed(false);
}
