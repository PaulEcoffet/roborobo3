//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2OPPORTUNITY_H
#define ROBOROBO3_COOPFIXED2OPPORTUNITY_H


#include "ext/World/RoundObject.h"
#include <set>

class CoopFixed2Opportunity: public RoundObject
{
public:
    explicit CoopFixed2Opportunity(int __id);
    void step() override;
    const std::set<int> & getNearbyRobotIndexes() const;
    void registerNewRobots();

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const;

    std::string inspect(std::string prefix) override;

protected:
    std::set<int> newRobotsOnOppThisTurn;
    std::set<int> robotsOnOppLastTurn;

    void updateColor();
};


#endif //ROBOROBO3_COOPFIXED2OPPORTUNITY_H
