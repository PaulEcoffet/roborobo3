//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PARTNERCHOICEWORLDMODEL_H
#define ROBOROBO3_PARTNERCHOICEWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>

class PartnerChoiceWorldModel : public RobotWorldModel
{
public:
    PartnerChoiceWorldModel();
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


#endif //ROBOROBO3_PARTNERCHOICEWORLDMODEL_H
