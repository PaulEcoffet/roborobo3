/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */


#ifndef COOPOPPORTUNITY2MAXAGENTOBSERVER_H
#define COOPOPPORTUNITY2MAXAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxSharedData.h"


#include <iomanip>

class CoopOpportunity2MaxAgentObserver : public AgentObserver
{
public:
    CoopOpportunity2MaxAgentObserver(RobotWorldModel *wm);

    ~CoopOpportunity2MaxAgentObserver();

    virtual void reset();

    virtual void stepPre();

    void logStats();

};

#endif

