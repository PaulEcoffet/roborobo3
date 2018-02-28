//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_FASTWANDEREROPPORTUNITY_H
#define ROBOROBO3_FASTWANDEREROPPORTUNITY_H


#include "World/RoundObject.h"
#include <set>

class FastWandererOpportunity: public RoundObject
{
public:
    explicit FastWandererOpportunity(int __id);
    void step() override;
    void setCoopValue(double coop);
    const std::set<int> & getNearbyRobotIndexes() const;
    void clearNearbyRobotIndexes();

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const;
    double getCoop() const;

    std::string inspect(std::string prefix="") override;

protected:
    double m_coop;
    std::set<int> nearbyRobotIndexes;

    void updateColor();
};


#endif //ROBOROBO3_FASTWANDEREROPPORTUNITY_H
