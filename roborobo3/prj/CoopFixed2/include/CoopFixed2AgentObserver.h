/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_COOPFIXED2AGENTOBSERVER_H
#define ROBOROBO3_COOPFIXED2AGENTOBSERVER_H


#include "core/Observers/AgentObserver.h"
#include "CoopFixed2WorldModel.h"

class CoopFixed2AgentObserver : public AgentObserver
{
public:
    CoopFixed2AgentObserver(RobotWorldModel *wm);
    ~CoopFixed2AgentObserver() override;

    void step() override;
    void reset() override;

protected:
    CoopFixed2WorldModel *m_wm;
};


#endif //ROBOROBO3_COOPFIXED2AGENTOBSERVER_H
