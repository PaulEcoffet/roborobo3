/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */


#ifndef COOPFIXED2AGENTOBSERVER_H
#define COOPFIXED2AGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"


#include <iomanip>

class CoopFixed2AgentObserver : public AgentObserver
{
public:
	CoopFixed2AgentObserver(RobotWorldModel *wm);
	~CoopFixed2AgentObserver();

	virtual void reset();
	virtual void step();
    
    void logStats();

};

#endif

