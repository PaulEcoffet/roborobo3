#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H


#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"

#include "World/CircleObject.h"

#include <set>

class MovingObject : public CircleObject
{
    
private:
    
    std::set<int> _nearbyRobots; // robots that are in the footprint in this iteration

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
