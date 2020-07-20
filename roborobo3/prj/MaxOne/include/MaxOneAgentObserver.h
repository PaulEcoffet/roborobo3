/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_MAXONEAGENTOBSERVER_H
#define ROBOROBO3_MAXONEAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "MaxOneWorldModel.h"

class MaxOneAgentObserver : public AgentObserver
{
public:
    MaxOneAgentObserver(RobotWorldModel *wm);

    ~MaxOneAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;



protected:
    MaxOneWorldModel *m_wm;

};


#endif //ROBOROBO3_MAXONEAGENTOBSERVER_H
