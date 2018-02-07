//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_CORRECTREPARTITIONOPPORTUNITY_H
#define ROBOROBO3_CORRECTREPARTITIONOPPORTUNITY_H


#include "ext/World/RoundObject.h"
#include <set>

class CorrectRepartitionOpportunity: public RoundObject
{
public:
    explicit CorrectRepartitionOpportunity(int __id);
    void step() override;
    const std::set<int> & getNearbyRobotIndexes() const;
    void clearNearbyRobotIndexes();

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const;

    std::string inspect(std::string prefix="") override;

protected:
    std::set<int> nearbyRobotIndexes;

    void updateColor();

    int prev_nb = 0;
};


#endif //ROBOROBO3_CORRECTREPARTITIONOPPORTUNITY_H
