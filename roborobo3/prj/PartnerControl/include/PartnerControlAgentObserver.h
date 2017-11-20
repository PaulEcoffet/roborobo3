/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_PARTNERCONTROLAGENTOBSERVER_H
#define ROBOROBO3_PARTNERCONTROLAGENTOBSERVER_H


#include "core/Observers/AgentObserver.h"
#include "PartnerControlWorldModel.h"

class PartnerControlAgentObserver : public AgentObserver
{
public:
    PartnerControlAgentObserver(RobotWorldModel *wm);
    ~PartnerControlAgentObserver() override;

    void step() override;
    void reset() override;

protected:
    PartnerControlWorldModel *m_wm;
};


#endif //ROBOROBO3_PARTNERCONTROLAGENTOBSERVER_H
