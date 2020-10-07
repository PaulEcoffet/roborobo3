/*
 *  SquareObject.h
 *  roborobo
 *
 *  Created by Nicolas on 26/4/2014.
 *
 */

#ifndef SQUAREOBJECT_H
#define SQUAREOBJECT_H

#include "World/PhysicalObject.h"

class SquareObject : public PhysicalObject
{

protected:


    int _solid_w;
    int _solid_h;
    int _soft_w;
    int _soft_h;

public :

    explicit SquareObject(int _id); // use PhysicalObjectFactory instead!
    virtual ~SquareObject() = default;

    bool canRegister() override; // test if register object is possible (use both shape or footprints)
    void registerObject() override; // register object in the world (write images)
    void unregisterObject() override; // unregister object in the world (write blank pixels)
    void show(SDL_Surface *surface) override;

    void hide() override;    // wrt. screen-rendering

    /* add default behavior for callbacks */

    void isTouched(int _id) override
    {};

    void isWalked(int _id) override
    {};

    void isPushed(int _id, std::tuple<double, double> _speed) override
    {};

};

#endif
