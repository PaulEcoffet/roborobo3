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
    int dx = (rand() % 3) - 1;
    int dy = (rand() % 3) - 1;
    
    unregisterObject();
    hide();
    if (canRegister(_xCenterPixel+dx, _yCenterPixel+dy))
    {
        _xCenterPixel += dx;
        _yCenterPixel += dy;
    }
    registerObject();

//	printf("Moving object %d a little bit, now at %d %d.\n",
//			_id, _xCenterPixel, _yCenterPixel);
	stepPhysicalObject();
	//TODO: probably add stuff here
}

void MovingObject::display() {
//	printf("Displaying moving object #%d\n", _id);
	CircleObject::display();
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
