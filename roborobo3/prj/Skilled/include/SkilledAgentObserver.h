/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_SKILLEDAGENTOBSERVER_H
#define ROBOROBO3_SKILLEDAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "SkilledWorldModel.h"

class SkilledAgentObserver : public AgentObserver {
public:
    SkilledAgentObserver(RobotWorldModel *wm);

    ~SkilledAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;


protected:
    SkilledWorldModel *m_wm;

};


#endif //ROBOROBO3_SKILLEDAGENTOBSERVER_H
