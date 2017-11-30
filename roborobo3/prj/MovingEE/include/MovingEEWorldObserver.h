/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */





#ifndef MOVINGEEWORLDOBSERVER_H
#define MOVINGEEWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingEE/include/MovingEESharedData.h"
#include "TemplateEE/include/TemplateEEWorldObserver.h"

//class World;

class MovingEEWorldObserver : public TemplateEEWorldObserver
{
public:
    MovingEEWorldObserver(World *world);
    ~MovingEEWorldObserver();
    
    virtual void stepPre();

protected:    
    virtual void monitorPopulation( bool localVerbose = true );
};

#endif
