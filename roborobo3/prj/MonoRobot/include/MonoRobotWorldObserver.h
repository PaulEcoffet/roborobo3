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
    
    int _activeObjects[2]; // which 2 objects of the 4 are active
    int _fakeRobotObject; // the object the fake robot is on

    std::ofstream _logFile;
    LogManager *_logManager; // Our own little logfile
    
public:
    MonoRobotWorldObserver(World *world);
    ~MonoRobotWorldObserver();
    
    
    virtual void reset();
    virtual void step();
    void stepGeneration();
    void resetObjects(); // reset which objects are active, etc.
    void resetLandmarks();

    bool objectIsActive(int __objectId); // tell if an object is active
    
    int getFakeRobotObject() { return _fakeRobotObject; }
    
    virtual int getGenerationItCount() { return _generationItCount; }

};

#endif
