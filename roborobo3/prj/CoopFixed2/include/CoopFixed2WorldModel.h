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
    std::deque<double> lastCommonKnowledgeReputation;

    double meanLastTotalInvest();

    double getCoop(bool trueValue = false) const;

    double meanLastOwnInvest();

    double meanLastCommonKnowledgeReputation();

    void appendOwnInvest(const double invest);

    void appendTotalInvest(const double invest);

    void appendToCommonKnowledgeReputation(const double d);

    void updateOtherReputation(int robid, double invest);

    bool isPlaying();

    double getOtherReputation(int robid);

    int getNbPlays(int robid);

    void initOtherReputations();

    double nbOnOpp = 0;
    bool onOpportunity;
    double selfA;

    void setNewSelfA();

    int arrival;
    CoopFixed2Opportunity *opp = nullptr;

    bool fake;
    double fakeCoef;

    std::vector<double> otherReputations;
    std::vector<int> nbPlays;

    void reset();

    bool teleport;
    double punishment;
    double spite;
    bool toBeTeleported = false;
    int prevopp = -1;
};


#endif //ROBOROBO3_COOPFIXED2WORLDMODEL_H
