//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2WORLDMODEL_H
#define ROBOROBO3_COOPFIXED2WORLDMODEL_H


#include <core/WorldModels/RobotWorldModel.h>
#include <deque>

class CoopFixed2WorldModel : public RobotWorldModel
{
public:
    CoopFixed2WorldModel();
    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;

    double meanLastTotalInvest();
    double meanLastOwnInvest();

    void appendOwnInvest(const double invest);
    void appendTotalInvest(const double invest);

    bool onOpportunity;
    double selfA;
    void setNewSelfA();


protected:
    unsigned int memorySize = 20;

};


#endif //ROBOROBO3_COOPFIXED2WORLDMODEL_H
