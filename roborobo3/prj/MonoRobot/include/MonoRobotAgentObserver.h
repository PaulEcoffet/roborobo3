/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MONOROBOTAGENTOBSERVER_H
#define MONOROBOTAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MonoRobot/include/MonoRobotSharedData.h"

#include <iomanip>

class MonoRobotAgentObserver : public AgentObserver
{
	public:
		MonoRobotAgentObserver(RobotWorldModel *wm);
		~MonoRobotAgentObserver();

		virtual void reset();
		virtual void step();

};

#endif

