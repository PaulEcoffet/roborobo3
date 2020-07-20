/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_CORRECTREPARTITIONAGENTOBSERVER_H
#define ROBOROBO3_CORRECTREPARTITIONAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "CorrectRepartitionWorldModel.h"

class CorrectRepartitionAgentObserver : public AgentObserver
{
public:
    CorrectRepartitionAgentObserver(RobotWorldModel *wm);

    ~CorrectRepartitionAgentObserver() override;

    void stepPre() override;

    void reset() override;

protected:
    CorrectRepartitionWorldModel *m_wm;
};


#endif //ROBOROBO3_CORRECTREPARTITIONAGENTOBSERVER_H
