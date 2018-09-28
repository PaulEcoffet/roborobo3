//
// Created by paul on 30/10/17.
//

#include "CorrectRepartition/include/CorrectRepartitionAgentObserver.h"

CorrectRepartitionAgentObserver::CorrectRepartitionAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<CorrectRepartitionWorldModel*>(wm);
}

CorrectRepartitionAgentObserver::~CorrectRepartitionAgentObserver() = default;


void CorrectRepartitionAgentObserver::stepPre()
{
    AgentObserver::stepPre();
    if (not m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

void CorrectRepartitionAgentObserver::reset()
{
    AgentObserver::reset();
}
