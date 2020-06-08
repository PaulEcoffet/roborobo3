//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_NEGOCIATEGYMWORLDMODEL_H
#define ROBOROBO3_NEGOCIATEGYMWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include <vector>
#include "NegociateGymOpportunity.h"
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "Utilities/Misc.h"

class NegociateGymWorldModel : public RobotWorldModel
{
public:
    NegociateGymWorldModel();

    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;
    std::deque<double> lastCommonKnowledgeReputation;

    std::vector<double> inputs;
    std::vector<double> actions;

    const std::vector<double> &getInputs() const;

    void setInputs(const std::vector<double> &inputs);

    const std::vector<double> &getActions() const;

    void setActions(const std::vector<double> &actions);

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
    NegociateGymOpportunity *opp = nullptr;

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
    int lastvisit = 0;
    bool seeking = true;

    void updateInputs();

protected:
    static std::vector<std::string> inputnames;
};


#endif //ROBOROBO3_NEGOCIATEGYMWORLDMODEL_H
