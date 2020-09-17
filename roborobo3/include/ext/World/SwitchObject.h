/*
 *  SwitchObject.h
 *  roborobo
 *
 *  Created by Nicolas on 25/4/2014.
 *
 */

#ifndef SWITCHOBJECT_H
#define SWITCHOBJECT_H

#include "World/CircleObject.h"

class SwitchObject : public CircleObject
{

private:
    int sendMessageTo;

public :

    SwitchObject(int __id); // use PhysicalObjectFactory instead!
    ~SwitchObject()
    {
    }

    void step() override;

    void isTouched(int __idAgent) override; // callback, triggered by agent
    void isWalked(int __idAgent) override; // callback, triggered by agent
    void
    isPushed(int __id, std::tuple<double, double> __speed) override; // callback, triggered by collision w/ agent/object
};

#endif
