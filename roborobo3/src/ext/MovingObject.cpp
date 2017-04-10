#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include "../prj/MovingNS/include/MovingNSController.h"

#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
    setType(5);
}

void MovingObject::step()
{
    CircleObject::step(); //handles movement, and sets _didMove
    for (auto robotID: _nearbyRobots)
    {
        Robot *robot = gWorld->getRobot(robotID);
//        MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
//        ctl->wasNearObject(_didMove);
    }
    _nearbyRobots.clear();
}

void MovingObject::show() {
    //	printf("Displaying moving object #%d\n", _id);
    CircleObject::show();
}

void MovingObject::isPushed( int __idAgent, Point2d __speed)
{
    CircleObject::isPushed(__idAgent, __speed);
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
    _nearbyRobots.insert(__idAgent);
}
