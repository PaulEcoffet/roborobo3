/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_PARTNERCHOICEAGENTOBSERVER_H
#define ROBOROBO3_PARTNERCHOICEAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "PartnerChoiceWorldModel.h"

class PartnerChoiceAgentObserver : public AgentObserver
{
public:
    PartnerChoiceAgentObserver(RobotWorldModel *wm);
    ~PartnerChoiceAgentObserver() override;

    void step() override;
    void reset() override;

protected:
    PartnerChoiceWorldModel *m_wm;
};


#endif //ROBOROBO3_PARTNERCHOICEAGENTOBSERVER_H
