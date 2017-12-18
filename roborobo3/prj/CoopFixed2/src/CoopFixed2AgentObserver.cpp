//
// Created by paul on 30/10/17.
//

#include "CoopFixed2/include/CoopFixed2AgentObserver.h"

CoopFixed2AgentObserver::CoopFixed2AgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<CoopFixed2WorldModel*>(wm);
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
}

void CoopFixed2AgentObserver::reset()
{
    AgentObserver::reset();
}
