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
#include "SingleGenome/include/SingleGenomeController.h"

#include <vector>

//class World;

class SingleGenomeWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();

    virtual void updateMonitoring();

    virtual void monitorPopulation(bool localVerbose = true);

    void stepEvaluation();

    void loadGenome();

    int _generationCount;
    int _generationItCount;

    LogManager *_fitnessLogManager;
    LogManager *_coopLogManager;

    std::vector<SingleGenomeController::genome> _genomes;
    std::vector<double> _fakeCoopValues;
    int _nbFakeRobots;
    int _fakeCoop;
    int _replica;
    int _genome;

public:
    SingleGenomeWorldObserver(World *world);

    ~SingleGenomeWorldObserver();

    virtual void reset();

    virtual void stepPre();

    LogManager *getCoopLogManager()
    { return _coopLogManager; }

    int getGenerationItCount()
    { return _generationItCount; }

    int getGenerationCount()
    { return _generationCount; }

    int getNbFakeRobots()
    { return _nbFakeRobots; }

    double getFakeCoop()
    { return _fakeCoopValues[_fakeCoop]; }

    int getGenome()
    { return _genome; }

    int getReplica()
    { return _replica; }
};

#endif
