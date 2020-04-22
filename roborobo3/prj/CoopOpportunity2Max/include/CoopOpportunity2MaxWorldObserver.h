/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef COOPOPPORTUNITY2MAXWORLDOBSERVER_H
#define COOPOPPORTUNITY2MAXWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxSharedData.h"

//class World;

class CoopOpportunity2MaxWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();

    virtual void updateMonitoring();

    virtual void monitorPopulation(bool localVerbose = true);

    int _generationCount;
    int _generationItCount;

    LogManager *_fitnessLogManager;
    LogManager *_genomeLogManager;

public:
    CoopOpportunity2MaxWorldObserver(World *world);

    ~CoopOpportunity2MaxWorldObserver();

    virtual void reset();

    virtual void stepPre();

    void stepEvaluation();

    virtual int getGenerationItCount()
    { return _generationItCount; }

    int getGenerationCount()
    { return _generationCount; }
};

#endif
