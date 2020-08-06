//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_PYNEGOTIATEWORLDMODEL_H
#define ROBOROBO3_PYNEGOTIATEWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include <pybind11/pybind11.h>
#include "PyNegotiateOpportunity.h"

namespace py = pybind11;


class PyNegotiateWorldModel : public RobotWorldModel
{
public:
    PyNegotiateWorldModel();

    std::deque<double> lastOwnInvest;
    std::deque<double> lastTotalInvest;
    std::deque<double> lastCommonKnowledgeReputation;


    double getCoop(bool trueValue = false) const;

    py::object getObservations() override;

    void setActions(const py::object &actions) override;

    bool getDone() override;

    double getReward() override;

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
    bool accept = false;
    double reward = 0;
    bool wasSeekerLastStep = true;
};


#endif //ROBOROBO3_PYNEGOTIATEWORLDMODEL_H
