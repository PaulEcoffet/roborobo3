//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PYNEGOTIATEWORLDMODEL_H
#define ROBOROBO3_PYNEGOTIATEWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include "PyNegotiateOpportunity.h"

class PyNegotiateWorldModel : public RobotWorldModel
{
public:
    PyNegotiateWorldModel();

    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;
    std::deque<double> lastCommonKnowledgeReputation;


    double getCoop(bool trueValue = false) const;


    bool isPlaying() const;

    double nbOnOpp = 0;
    bool onOpportunity;
    double selfA;

    void setNewSelfA();

    int arrival = 0;
    PyNegotiateOpportunity *opp = nullptr;

    bool fake;
    double fakeCoef;

    std::vector<int> nbPlays;

    void reset();

    bool teleport;
    double punishment;
    double spite;
    bool toBeTeleported = false;
    int lastvisit = 0;
    bool seeking = true;
    int prevopp = -1;
};


#endif //ROBOROBO3_PYNEGOTIATEWORLDMODEL_H
