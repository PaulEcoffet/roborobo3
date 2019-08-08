/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_NEGOCIATEAGENTOBSERVER_H
#define ROBOROBO3_NEGOCIATEAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "NegociateWorldModel.h"

class NegociateAgentObserver : public AgentObserver
{
public:
    NegociateAgentObserver(RobotWorldModel *wm);

    ~NegociateAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;


protected:
    NegociateWorldModel *m_wm;

};


#endif //ROBOROBO3_NEGOCIATEAGENTOBSERVER_H
