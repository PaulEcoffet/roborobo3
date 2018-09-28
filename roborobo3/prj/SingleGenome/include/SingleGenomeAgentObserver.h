/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef SINGLEGENOMEAGENTOBSERVER_H
#define SINGLEGENOMEAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "SingleGenome/include/SingleGenomeSharedData.h"

#include <iomanip>

class SingleGenomeAgentObserver : public AgentObserver
{
public:
    SingleGenomeAgentObserver(RobotWorldModel *wm);
    ~SingleGenomeAgentObserver();

    virtual void reset();
    virtual void stepPre();
    
    void logStats();

};

#endif

