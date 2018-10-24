//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2WORLDMODEL_H
#define ROBOROBO3_COOPFIXED2WORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include "CoopFixed2Opportunity.h"

class CoopFixed2WorldModel : public RobotWorldModel
{
public:
    CoopFixed2WorldModel();
    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;
    std::deque<double> lastReputation;

    double meanLastTotalInvest();
    double meanLastOwnInvest();
    double meanLastReputation();
    void appendOwnInvest(const double invest);

    void appendTotalInvest(const double invest);
    void appendToReputation(const double d);

    double nbOnOpp = 0;
    bool onOpportunity;
    double selfA;
    void setNewSelfA();
    int arrival;
    CoopFixed2Opportunity* opp;

    bool fake;
    double fakeCoef;

    void reset();

    bool teleport;
    double punishment;
    double spite;
};


#endif //ROBOROBO3_COOPFIXED2WORLDMODEL_H
