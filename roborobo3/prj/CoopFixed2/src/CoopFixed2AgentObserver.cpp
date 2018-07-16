//
// Created by paul on 30/10/17.
//

#include <core/RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2AgentObserver.h"

CoopFixed2AgentObserver::CoopFixed2AgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<CoopFixed2WorldModel*>(wm);
    _wm = wm;
}

CoopFixed2AgentObserver::~CoopFixed2AgentObserver() = default;


void CoopFixed2AgentObserver::step()
{
    AgentObserver::step();

    if (not m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }

    int targetIndex = m_wm->getGroundSensorValue();
    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset);
    }
}

void CoopFixed2AgentObserver::reset()
{
    AgentObserver::reset();
}
