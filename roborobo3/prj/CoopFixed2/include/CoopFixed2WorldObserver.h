/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef COOPFIXED2WORLDOBSERVER_H
#define COOPFIXED2WORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"
#include <set>

//class World;

class CoopFixed2WorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;


protected:

    LogManager *_fitnessLogManager;
    LogManager *_genomeLogManager;
    
public:
    CoopFixed2WorldObserver(World *world);
    ~CoopFixed2WorldObserver() override;

    void reset() override;

    void step() override;

    void stepEvaluation();
    
    virtual int getGenerationItCount() { return _generationItCount; }
    
    int getGenerationCount() { return _generationCount; }

    void addRobotToTeleport(int i);

    std::set<int> _robotsToTeleport;

    void teleportRobots(std::set<int> const& robotsToTeleport) const;

    void computeOpportunityImpact() const;
};

#endif
