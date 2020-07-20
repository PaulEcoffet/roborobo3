//
// Created by paul on 30/10/17.
//

#include "PartnerChoice/include/PartnerChoiceAgentObserver.h"

PartnerChoiceAgentObserver::PartnerChoiceAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<PartnerChoiceWorldModel *>(wm);
}

PartnerChoiceAgentObserver::~PartnerChoiceAgentObserver() = default;


void PartnerChoiceAgentObserver::stepPre()
{
    AgentObserver::stepPre();
    if (not m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

void PartnerChoiceAgentObserver::reset()
{
    AgentObserver::reset();
}
