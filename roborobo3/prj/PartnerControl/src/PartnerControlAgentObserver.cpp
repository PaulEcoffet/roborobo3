//
// Created by paul on 30/10/17.
//

#include "PartnerControl/include/PartnerControlAgentObserver.h"

PartnerControlAgentObserver::PartnerControlAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<PartnerControlWorldModel*>(wm);
}

PartnerControlAgentObserver::~PartnerControlAgentObserver() = default;


void PartnerControlAgentObserver::step()
{
    AgentObserver::step();
    if (not m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

void PartnerControlAgentObserver::reset()
{
    AgentObserver::reset();
}
