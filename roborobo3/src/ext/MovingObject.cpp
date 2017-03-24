#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
    setType(5);
}

void MovingObject::step()
{
    _hitWall = false;
    if (_impulses.size() > 0)
    {
        //       printf("[DEBUG] Moving object %d\n", _id);
        double impXtot = 0, impYtot = 0, vx, vy, ux, uy;
        
        for (auto& imp : _impulses) {
            // We only want the component of the speed normal to the centers of mass
            // v: agent speed vector, u: agent-object vector
            if (imp.first >= gRobotIndexStartOffset) { // robot
                // impulses are in polar form, and angles in degrees
                Robot *robot = gWorld->getRobot(imp.first-gRobotIndexStartOffset);
                vx = imp.second.x*cos(imp.second.y * M_PI / 180.0);
                vy = imp.second.x*sin(imp.second.y * M_PI / 180.0);
                ux = (double)_xCenterPixel - robot->getWorldModel()->getXReal();
                uy = (double)_yCenterPixel - robot->getWorldModel()->getYReal();
                
            }
            else // other object
            {
                // impulse is cartesian and not polar here
                PhysicalObject *object = gPhysicalObjects[imp.first];
                vx = imp.second.x;
                vy = imp.second.y;
                ux = (double)_xCenterPixel - object->getPosition().x;
                uy = (double)_yCenterPixel - object->getPosition().y;
            }
            double sqnorm = ux*ux + uy*uy;
            impXtot += (vx*ux+vy*uy)*ux/sqnorm;
            impYtot += (vx*ux+vy*uy)*uy/sqnorm;
        }
        _impulses.clear();
        
        _xDesiredSpeed = impXtot;
        _yDesiredSpeed = impYtot;
        
        int dx = roundAwayFromZero(impXtot);
        int dy = roundAwayFromZero(impYtot);
        
        int newX = _xCenterPixel + dx;
        int newY = _yCenterPixel + dy;
        //       printf("[DEBUG] Impulses: %lf->%d (x) %lf->%d (y).\n", impXtot, dx,
        //              impYtot, dy);
        
        unregisterObject();
        hide();
        
        if (canRegister(newX, newY))
        {
            _xCenterPixel = newX;
            _yCenterPixel = newY;
        }
        if (_hitWall) { // reappear somewhere else
            registered = false;
            _visible = false;
        }
        else
            registerObject();
    }
    stepPhysicalObject();
}

void MovingObject::show() {
    //	printf("Displaying moving object #%d\n", _id);
    CircleObject::show();
}

void MovingObject::isPushed( int __idAgent, Point2d __speed)
{
    if (_impulses.count(__idAgent) == 0) {
//        printf("[DEBUG] object %d is being pushed by agent %d.\n", _id, __idAgent);
        _impulses.insert(std::pair<int, Point2d>(__idAgent, __speed));
        // If the agent is a robot, tell it we pushed something
        if (__idAgent >= gRobotIndexStartOffset) {
            Robot* robot = gWorld->getRobot(__idAgent-gRobotIndexStartOffset);
            robot->getWorldModel()->setPushed(true);
        }
    }
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
    // just make the object disappear for now
    regrowTime = regrowTimeMax;
    
    registered = false;
    unregisterObject();
    hide();
    _visible = false;
}
