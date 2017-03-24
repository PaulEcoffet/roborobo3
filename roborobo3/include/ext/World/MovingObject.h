#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H


#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"

#include "World/CircleObject.h"

#include <map>

class MovingObject : public CircleObject
{

private:
    
    // the impulses given to the object by nearby robots in the current time step, or other objects in the former
    // The vectors are given in polar form
    std::map<int, Point2d> _impulses;

public:

	MovingObject( int __id );
	~MovingObject() {}

	void show();

	void step();
    void isPushed( int __idAgent, Point2d __speed) ; // callback
	void isTouched( int __idAgent ); // callback, triggered by agent
	void isWalked( int __idAgent ); // callback, triggered by agent

};

#endif
