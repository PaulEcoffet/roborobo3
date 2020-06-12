/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-30
*/

#ifndef ROBOROBO3_PYFASTWANDERERAGENTOBSERVER_H
#define ROBOROBO3_PYFASTWANDERERAGENTOBSERVER_H


#include <WorldModels/WorldModel.h>
#include "Observers/AgentObserver.h"

class pyFastWandererAgentObserver : public AgentObserver
{
public:
    pyFastWandererAgentObserver(RobotWorldModel *wm);
    ~pyFastWandererAgentObserver() override;

    void stepPre() override;
    void reset() override;

protected:
    RobotWorldModel *m_wm;
};


#endif //ROBOROBO3_PYFASTWANDERERAGENTOBSERVER_H
