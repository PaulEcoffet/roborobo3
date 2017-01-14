/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef TEMPLATEEEWORLDOBSERVER_H
#define TEMPLATEEEWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "TemplateEE/include/TemplateEESharedData.h"

//class World;

class TemplateEEWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;
    
public:
    TemplateEEWorldObserver(World *world);
    ~TemplateEEWorldObserver();
    
    virtual void reset();
    virtual void step();
    
    virtual  int getGenerationItCount() { return _generationItCount; }

};

#endif
