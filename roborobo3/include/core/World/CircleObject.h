/*
 *  CircleObject.h
 *  roborobo
 *
 *  Created by Nicolas on 25/4/2014.
 *
 */

#ifndef CIRCLEOBJECT_H
#define CIRCLEOBJECT_H

#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"

#include "World/PhysicalObject.h"

#include <map>

class CircleObject : public PhysicalObject
{

protected:
    
    double _radius; // radius. In pixels.
    double _footprintRadius; // radius of footprint, accessible to ground sensors. In pixels.
    
    // the impulses given to the object by nearby robots in the current time step, or other objects in the former
    std::map<int, Point2d> _impulses;
    
    // the speed we expect to move at after computing collisions, which we need to be able to tell other objects
    // in polar coordinates
    double _desiredSpeedOrientation;
    double _desiredLinearSpeed;
    
    bool _hitWall;
    
public:
    
    CircleObject( int __id ); // use PhysicalObjectFactory instead!
    ~CircleObject() { }

    bool canRegister(); // can we register at current position
    bool canRegister( int __x, int __y ); // can we register the object at that position
    void registerObject(); // register object in the world (write images)
    void unregisterObject(); // unregister object in the world (write blank pixels)
    void show(); // wrt. screen-rendering
    void hide();    // wrt. screen-rendering
    
    void step(); // handles dynamics
    
    void isPushed( int __idAgent, Point2d __speed ); // register we've been pushed
    
    void isTouched( int __idAgent );
    void isWalked( int __idAgent );
    
};

#endif
