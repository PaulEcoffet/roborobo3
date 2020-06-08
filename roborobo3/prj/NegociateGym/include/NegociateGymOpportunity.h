//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_NEGOCIATEGYMOPPORTUNITY_H
#define ROBOROBO3_NEGOCIATEGYMOPPORTUNITY_H


#include "World/RoundObject.h"
#include <set>

class NegociateGymOpportunity : public RoundObject
{
public:
    explicit NegociateGymOpportunity(int __id);

    void step() override;

    virtual const std::vector<int> &getNearbyRobotIndexes() const;

    virtual void registerNewRobots();

    void isWalked(int id) override;

    virtual int getNbNearbyRobots() const;

    virtual int getNbNewNearbyRobots() const;

    virtual void kill();

    void isPushed(int rid, std::tuple<double, double> speed) override;

    std::string inspect(std::string prefix) override;

    bool hasBeenTouched(int rid) const;


    double curInv = 0;
    double curA = 0;
    int lifeid;
    double lifeExpectancy;


protected:
    std::vector<int> nearbyRobotIndexes;
    std::vector<int> newNearbyRobotIndexes;
    std::set<int> touchingrobotsSet;

    virtual void updateColor();

};


#endif //ROBOROBO3_NEGOCIATEGYMOPPORTUNITY_H
