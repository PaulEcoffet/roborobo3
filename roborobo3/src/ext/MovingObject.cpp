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

    double movement = sqrt((oldX-_xReal)*(oldX-_xReal) + (oldY-_yReal)*(oldY-_yReal));
	int nbRobots = static_cast<int>(_nearbyRobots.size());
	double coeff = 1.0/(1.0+pow(nbRobots-2, 2)); // \frac{1}{1+(nbRobots-2)^2}
	double gain = movement * coeff;

    for (auto robotID: _nearbyRobots)
    {
        Robot *robot = gWorld->getRobot(robotID);
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
    CircleObject::isPushed(__idAgent, __speed);
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
    _nearbyRobots.insert(__idAgent);
}
