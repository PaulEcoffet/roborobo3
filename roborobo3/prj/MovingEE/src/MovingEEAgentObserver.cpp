/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "MovingEE/include/MovingEEAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "MovingEE/include/MovingEEController.h"
#include <cmath>
#include "MovingEE/include/MovingEEWorldObserver.h"
#include <string>

MovingEEAgentObserver::MovingEEAgentObserver( RobotWorldModel *wm ) : TemplateEEAgentObserver ( wm )
{
    // superclass constructor called before
}

MovingEEAgentObserver::~MovingEEAgentObserver()
{
    // superclass destructor called before
}

/*
 * Manage pushing items.
 * If we pushed something, increase our fitness.
 *
 */
void MovingEEAgentObserver::step()
{
    // * update fitness (if needed)
    if ( _wm->isAlive() && _wm->getDidPush() )
    {
        _wm->_fitnessValue = _wm->_fitnessValue + 1;
    }
    _wm->setTriedPushing(false);
    _wm->setDidPush(false);

    TemplateEEAgentObserver::step();
}
