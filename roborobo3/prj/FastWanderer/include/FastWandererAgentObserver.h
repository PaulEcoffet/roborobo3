/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_FASTWANDERERAGENTOBSERVER_H
#define ROBOROBO3_FASTWANDERERAGENTOBSERVER_H


#include <core/WorldModels/WorldModel.h>
#include "core/Observers/AgentObserver.h"

class FastWandererAgentObserver : public AgentObserver
{
public:
    FastWandererAgentObserver(RobotWorldModel *wm);
    ~FastWandererAgentObserver() override;

    void step() override;
    void reset() override;

protected:
    RobotWorldModel *m_wm;
};


#endif //ROBOROBO3_FASTWANDERERAGENTOBSERVER_H
