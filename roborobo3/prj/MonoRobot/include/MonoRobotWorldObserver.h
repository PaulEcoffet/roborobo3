/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MONOROBOTWORLDOBSERVER_H
#define MONOROBOTWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MonoRobot/include/MonoRobotSharedData.h"

//class World;

class MonoRobotWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;
    
    std::ofstream _logFile;
    LogManager *_logManager; // Our own little logfile
    
public:
    MonoRobotWorldObserver(World *world);
    ~MonoRobotWorldObserver();
    
    virtual void reset();
    virtual void step();
    
    virtual int getGenerationItCount() { return _generationItCount; }

};

#endif
