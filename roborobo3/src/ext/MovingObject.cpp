#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
	setType(5);	
}

bool MovingObject::canRegister( int x, int y )
{
    int oldX = _xCenterPixel;
    int oldY = _yCenterPixel;
    _xCenterPixel = x;
    _yCenterPixel = y;
    bool canReg = CircleObject::canRegister();
    _xCenterPixel = oldX;
    _yCenterPixel = oldY;
    return canReg;
}

void MovingObject::step()
{
    if (_robotImpulses.size() > 0)
    {
 //       printf("[DEBUG] Moving object %d\n", _id);
        double impXtot = 0, impYtot = 0;
        
        // impulses are in polar form, and angles in degrees
        for (auto& imp : _robotImpulses) {
            //We only want the normal component of the speed wrt the centers of mass
            
            double vx = imp.second.x*cos(imp.second.y * M_PI / 180.0);
            double vy = imp.second.x*sin(imp.second.y * M_PI / 180.0);
            
            Robot *robot = gWorld->getRobot(imp.first);
            double ux = (double)_xCenterPixel - robot->getWorldModel()->getXReal();
            double uy = (double)_yCenterPixel - robot->getWorldModel()->getYReal();
            double sqnorm = ux*ux + uy*uy;
            
            double impX = (vx*ux+vy*uy)*ux/sqnorm;
            double impY = (vx*ux+vy*uy)*uy/sqnorm;
            
            impXtot += impX;
            impYtot += impY;
        }
        _robotImpulses.clear();

        int newX = _xCenterPixel + (int)lround(impXtot);
        int newY = _yCenterPixel + (int)lround(impYtot);
//        printf("[DEBUG] Impulses: %d (x) %d (y).\n", newX-_xCenterPixel, newY-_yCenterPixel);
        
        unregisterObject();
        hide();
        
        if (canRegister(newX, newY))
        {
            _xCenterPixel = newX;
            _yCenterPixel = newY;
        }
    
        registerObject();
    }
    stepPhysicalObject();
}

void MovingObject::display() {
//	printf("Displaying moving object #%d\n", _id);
	CircleObject::display();
}

void MovingObject::isPushed( int __idAgent, Point2d __speed)
{
//    printf("[DEBUG]: object %d is being pushed by agent %d.\n", _id, __idAgent);
    if (_robotImpulses.count(__idAgent) == 0)
        _robotImpulses.insert(std::pair<int, Point2d>(__idAgent, __speed));
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