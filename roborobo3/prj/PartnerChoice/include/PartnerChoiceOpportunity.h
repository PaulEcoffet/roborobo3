//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PARTNERCHOICEOPPORTUNITY_H
#define ROBOROBO3_PARTNERCHOICEOPPORTUNITY_H


#include "ext/World/RoundObject.h"
#include <set>

class PartnerChoiceOpportunity: public RoundObject
{
public:
    explicit PartnerChoiceOpportunity(int __id);
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


#endif //ROBOROBO3_PARTNERCHOICEOPPORTUNITY_H
