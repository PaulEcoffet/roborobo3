//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLOPPORTUNITY_H
#define ROBOROBO3_PARTNERCONTROLOPPORTUNITY_H


#include "ext/World/RoundObject.h"
#include <set>

class PartnerControlOpportunity: public RoundObject
{
public:
    explicit PartnerControlOpportunity(int __id);
    void step() override;
    void setCoopValue(double coop);
    const std::set<int> & getNearbyRobotIndexes() const;
    void clearNearbyRobotIndexes();

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const;
    double getCoop() const;

protected:
    double m_coop;
    std::set<int> nearbyRobotIndexes;

    void updateColor();
};


#endif //ROBOROBO3_PARTNERCONTROLOPPORTUNITY_H
