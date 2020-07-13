/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include <core/RoboroboMain/roborobo.h>
#include "TemplateWander/include/TemplateWanderWorldObserver.h"
#include "World/World.h"


TemplateWanderWorldObserver::TemplateWanderWorldObserver( World *__world ) : WorldObserver( __world )
{
	_world = __world;
}

TemplateWanderWorldObserver::~TemplateWanderWorldObserver()
{
	// nothing to do.
}

void TemplateWanderWorldObserver::reset()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }
    int i = 0;
    for (auto object: gPhysicalObjects)
    {
        object->findRandomLocation();
        object->registerObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->registerRobot();
    }
}

void TemplateWanderWorldObserver::stepPre()
{
	// nothing to do.
}

void TemplateWanderWorldObserver::stepPost()
{
    // nothing to do.
}
