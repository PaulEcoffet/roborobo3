/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */

#ifndef TEMPLATEMOVINGAGENTOBSERVER_H
#define TEMPLATEMOVINGAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "WorldModels/RobotWorldModel.h"
#include "Observers/AgentObserver.h"

class TemplateMovingAgentObserver : public AgentObserver
{
public:
    TemplateMovingAgentObserver();

    TemplateMovingAgentObserver(RobotWorldModel *__wm);

    ~TemplateMovingAgentObserver();

    void reset();

    void stepPre();

};


#endif

