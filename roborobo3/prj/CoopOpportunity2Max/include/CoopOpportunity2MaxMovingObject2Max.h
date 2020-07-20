/*
 * MovingObject2Max.h
 *
 *  Created on: 9 oct. 2017
 *      Author: Paul Ecoffet
 */

#ifndef PRJ_COOPOPPORTUNITY2MAX_SRC_MOVINGOBJECT2MAX_H_
#define PRJ_COOPOPPORTUNITY2MAX_SRC_MOVINGOBJECT2MAX_H_

#include "World/MovingObject.h"

class CoopOpportunity2MaxMovingObject2Max : public MovingObject
{
public:
    CoopOpportunity2MaxMovingObject2Max(int __id);

    virtual ~CoopOpportunity2MaxMovingObject2Max();

    /**
     * isPushed is override to prevent robots from being more than 2 on an object
     */
    virtual void isPushed(int __idAgent, std::tuple<double, double> __speed);

    virtual void step();

protected:
    std::set<int> _prevNearbyRobots;

};

#endif /* PRJ_COOPOPPORTUNITY2MAX_SRC_MOVINGOBJECT2MAX_H_ */
