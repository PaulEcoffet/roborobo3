//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2OPPORTUNITY_H
#define ROBOROBO3_COOPFIXED2OPPORTUNITY_H


#include "World/RoundObject.h"
#include <set>

class CoopFixed2Opportunity: public RoundObject
{
public:
    explicit CoopFixed2Opportunity(int __id);
    void step() override;

    virtual const std::vector<int> & getNearbyRobotIndexes() const;
    virtual void registerNewRobots();

    void isWalked(int id) override;

    virtual int getNbNearbyRobots() const;

    std::string inspect(std::string prefix) override;
    double curInv = 0;
    double curA = 0;
    int lifeid;



protected:
    std::vector<int> nearbyRobotIndexes;
    std::vector<int> newNearbyRobotIndexes;
    double lifeExpectancy;

    virtual void updateColor();
};


#endif //ROBOROBO3_COOPFIXED2OPPORTUNITY_H
