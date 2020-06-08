/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_NEGOCIATEGYMAGENTOBSERVER_H
#define ROBOROBO3_NEGOCIATEGYMAGENTOBSERVER_H


#include "Observers/AgentObserver.h"
#include "NegociateGymWorldModel.h"

class NegociateGymAgentObserver : public AgentObserver
{
public:
    NegociateGymAgentObserver(RobotWorldModel *wm);

    ~NegociateGymAgentObserver() override;

    void stepPre() override;

    void reset() override;

    void stepPost() override;

    int getSeekTime();


protected:
    NegociateGymWorldModel *m_wm;

    int m_seekTime;

    void color_agent() const;

    void mark_walking_opp() const;

    void updateCameraInput();
};


#endif //ROBOROBO3_NEGOCIATEGYMAGENTOBSERVER_H
