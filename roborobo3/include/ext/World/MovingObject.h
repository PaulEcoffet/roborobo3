#ifndef MOVINGOBJECT_H
#define MOVINGOBJECT_H


#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"

#include "World/CircleObject.h"

#include <map>
#include <set>
#include <tuple>

class MovingObject : public CircleObject
{
    
private:
    
    std::set<int> _nearbyRobots; // robots that are in the footprint in this iteration
    
    // the impulses given to the object by nearby robots in the current time step, or other objects in the former
    std::map<int, std::tuple<double, double>> _impulses;
    std::map<int, double> _efforts; // remember how much each robot actually pushed us
    
    // the speed we expect to move at after computing collisions, which we need to be able to tell other objects
    // in polar coordinates
    double _desiredSpeedOrientation;
    double _desiredLinearSpeed;
    
    // the coordinates we'd like to end up in after we move
    // (don't go there is gStuckMovableObjects is set, or if we collide)
    double _desiredX;
    double _desiredY;
    
    bool _hitWall;
    bool _didMove;

public:

	MovingObject( int __id );
	~MovingObject() {}

	void show();
    
    bool canRegister(); // can we register at current position
    bool canRegisterStatic( Sint16 __x, Sint16 __y ); // can we register the object at that position (we're trying to create it there)
    bool canRegisterDynamic( Sint16 __x, Sint16 __y ); // we're trying to move there

	void step();
    void move(); // physically move
    void isPushed( int __idAgent, std::tuple<double, double> __speed ); // callback
	void isTouched( int __idAgent ); // callback, triggered by agent
	void isWalked( int __idAgent ); // callback, triggered by agent

};

#endif
