//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_CORRECTREPARTITIONWORLDMODEL_H
#define ROBOROBO3_CORRECTREPARTITIONWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>

class CorrectRepartitionWorldModel : public RobotWorldModel
{
public:
    CorrectRepartitionWorldModel();
    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;

    double meanLastTotalInvest();
    double meanLastOwnInvest();

    void appendOwnInvest(const double invest);
    void appendTotalInvest(const double invest);

    bool onOpportunity;
    int nbOnOpp = 0;

protected:
    unsigned int memorySize = 20;
};


#endif //ROBOROBO3_CORRECTREPARTITIONWORLDMODEL_H
