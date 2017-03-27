/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MOVINGNSWORLDOBSERVER_H
#define MOVINGNSWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingNS/include/MovingNSSharedData.h"

//class World;

class MovingNSWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;
    
public:
    MovingNSWorldObserver(World *world);
    ~MovingNSWorldObserver();
    
    virtual void reset();
    virtual void step();
    
    virtual  int getGenerationItCount() { return _generationItCount; }

};

#endif
