/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "Dummy/include/DummyAgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"


DummyAgentObserver::DummyAgentObserver( RobotWorldModel *wm )
{
    _wm = (RobotWorldModel*)wm;
    
}

DummyAgentObserver::~DummyAgentObserver()
{
    // nothing to do.
}

void DummyAgentObserver::reset()
{
    // nothing to do.
}

void DummyAgentObserver::stepPre()
{
    // Nothing to do.
}
