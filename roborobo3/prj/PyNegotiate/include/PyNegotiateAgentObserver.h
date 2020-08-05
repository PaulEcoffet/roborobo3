/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_PYNEGOTIATEAGENTOBSERVER_H
#define ROBOROBO3_PYNEGOTIATEAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "PyNegotiateWorldModel.h"

class PyNegotiateAgentObserver : public AgentObserver
{
public:
    PyNegotiateAgentObserver(RobotWorldModel *wm);

    ~PyNegotiateAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;

    int getSeekTime() const;


protected:
    PyNegotiateWorldModel *m_wm;

    int m_seekTime;

    void color_agent() const;

    void mark_walking_opp() const;
};


#endif //ROBOROBO3_PYNEGOTIATEAGENTOBSERVER_H
