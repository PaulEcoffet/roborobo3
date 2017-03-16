#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H


#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"

#include "World/CircleObject.h"

class MovingObject : public CircleObject
{

private:

public:

	MovingObject( int __id );
	~MovingObject() {}

	void display();
    
    bool canRegister( int x, int y ); //calls CircleObject::canRegister at the new x and y

	void step();
	void isTouched( int __idAgent ); // callback, triggered by agent
	void isWalked( int __idAgent ); // callback, triggered by agent

};

#endif
