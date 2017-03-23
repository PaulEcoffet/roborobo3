/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MOVINGEEAGENTOBSERVER_H
#define MOVINGEEAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingEE/include/MovingEESharedData.h"
#include <iomanip>
#include "TemplateEE/include/TemplateEEAgentObserver.h"

class MovingEEAgentObserver : public TemplateEEAgentObserver
{
	public:
		MovingEEAgentObserver(RobotWorldModel *wm);
		~MovingEEAgentObserver();
    
        virtual void step();
};

#endif

