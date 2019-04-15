/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_LIONAGENTOBSERVER_H
#define ROBOROBO3_LIONAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "LionWorldModel.h"

class LionAgentObserver : public AgentObserver
{
public:
    LionAgentObserver(RobotWorldModel *wm);

    ~LionAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;


protected:
    LionWorldModel *m_wm;

};


#endif //ROBOROBO3_LIONAGENTOBSERVER_H
