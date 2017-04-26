#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include "../prj/MovingNS/include/MovingNSController.h"
#include "../prj/MonoRobot/include/MonoRobotController.h"


#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
    setType(5);
}

void MovingObject::step()
{
    double oldX = _xReal, oldY = _yReal;

    CircleObject::step(); //handles movement, and sets _didMove

    double movement = sqrt((oldX-_desiredX)*(oldX-_desiredX) + (oldY-_desiredY)*(oldY-_desiredY));
	int nbRobots = static_cast<int>(_nearbyRobots.size());
	double coeff = 1.0/(1.0+pow(nbRobots-2, 2)); // \frac{1}{1+(nbRobots-2)^2}
	double gain = movement * coeff;
    
    if (gStuckMovableObjects) // get back into place!
    {
        _xReal = oldX;
        _yReal = oldY;
    }

    for (auto robotID: _nearbyRobots)
    {
        Robot *robot = gWorld->getRobot(robotID);
        // See ConfigurationLoader.cpp for a way to dynamically adjust to which project we're running
        MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
        ctl->wasNearObject(_didMove, gain, nbRobots);
    }
    _nearbyRobots.clear();
}

void MovingObject::show() {
    //	printf("Displaying moving object #%d\n", _id);
    CircleObject::show();
}

void MovingObject::isPushed( int __idAgent, Point2d __speed)
{
    printf("[DEBUG] Object %d is being pushed by agent %d\n", _id, __idAgent);
    CircleObject::isPushed(__idAgent, __speed);
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
    _nearbyRobots.insert(__idAgent);
}
