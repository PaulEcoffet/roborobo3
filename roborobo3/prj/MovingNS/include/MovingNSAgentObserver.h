/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MOVINGNSAGENTOBSERVER_H
#define MOVINGNSAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingNS/include/MovingNSSharedData.h"

#include <iomanip>

class MovingNSAgentObserver : public AgentObserver
{
public:
    MovingNSAgentObserver(RobotWorldModel *wm);

    ~MovingNSAgentObserver();

    virtual void reset();

    virtual void stepPre();

    void logStats();

};

#endif

