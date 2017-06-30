/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef SINGLEGENOMEWORLDOBSERVER_H
#define SINGLEGENOMEWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "SingleGenome/include/SingleGenomeSharedData.h"

//class World;

class SingleGenomeWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;
    
    LogManager *_fitnessLogManager;
    LogManager *_genomeLogManager;
    LogManager *_coopLogManager;
    
public:
    SingleGenomeWorldObserver(World *world);
    ~SingleGenomeWorldObserver();
    
    virtual void reset();
    virtual void step();
    void stepEvaluation();
    
    LogManager *getCoopLogManager() { return _coopLogManager; }
    
    virtual int getGenerationItCount() { return _generationItCount; }

    int getGenerationCount() { return _generationCount; }
};

#endif
