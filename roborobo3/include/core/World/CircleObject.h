/*
 *  CircleObject.h
 *  roborobo
 *
 *  Created by Nicolas on 25/4/2014.
 *
 */

#ifndef CIRCLEOBJECT_H
#define CIRCLEOBJECT_H

#include "World/PhysicalObject.h"

#include <map>

class CircleObject : public PhysicalObject
{

protected:
    
    int _radius; // radius. In pixels.
    int _footprintRadius; // radius of footprint, accessible to ground sensors. In pixels.
    
public:
    
    CircleObject( int __id ); // use PhysicalObjectFactory instead!
    ~CircleObject() { }

    bool canRegister(); // can we register at current position
    void registerObject(); // register object in the world (write images)
    void unregisterObject(); // unregister object in the world (write blank pixels)
    void show(SDL_Surface *surface = gScreen); // wrt. screen-rendering
    void hide();    // wrt. screen-rendering
        
};

#endif
