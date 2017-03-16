#include "World/EnergyItem.h"

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
	_xCenterPixel += 5;
	_yCenterPixel += 5;
	printf("Moving object %d a little bit, now at %d %d.\n", 
			_id, _xCenterPixel, _yCenterPixel);
	stepPhysicalObject();
	//TODO: probably add stuff here
}

void MovingObject::display() {
	printf("Displaying moving object #%d\n", _id);
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
