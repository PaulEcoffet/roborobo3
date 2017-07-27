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
    int _evaluationCount;
    int _evaluationItCount;
    
    LogManager *_statsLogManager; // Our own little logfile
    LogManager *_genomeLogManager;
    LogManager *_effortLogManager; // log effort values for each iteration and each robot so we can see if they converge to the ESS

    
public:
    MonoRobotWorldObserver(World *world);
    ~MonoRobotWorldObserver();
    
    
    virtual void reset();
    virtual void step();
    void stepEvaluation( bool __newGeneration );
    void resetObjects(); // reset which objects are active, etc.
    void resetLandmarks();
    
    virtual int getGenerationItCount() { return _evaluationItCount; }
    
};

#endif
