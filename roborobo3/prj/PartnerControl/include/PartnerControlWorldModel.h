//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLWORLDMODEL_H
#define ROBOROBO3_PARTNERCONTROLWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>

class PartnerControlWorldModel : public RobotWorldModel
{
public:
    PartnerControlWorldModel();
    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;

    double meanLastTotalInvest();
    double meanLastOwnInvest();

    void appendOwnInvest(const double invest);
    void appendTotalInvest(const double invest);

    bool onOpportunity;

protected:
    unsigned int memorySize = 20;
};


#endif //ROBOROBO3_PARTNERCONTROLWORLDMODEL_H
