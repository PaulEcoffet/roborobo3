//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_LIONWORLDMODEL_H
#define ROBOROBO3_LIONWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include "LionOpportunity.h"

class LionOpportunity;

class LionWorldModel : public RobotWorldModel
{
public:
    LionWorldModel();

    double nbOnOpp = 0;
    bool onOpportunity;
    double selfA;

    void setNewSelfA();

    int arrival{};
    LionOpportunity *opp = nullptr;

    bool fake;
    bool ingame = false;
    double fakeCoef;

    void reset();

    int teleport = -1;

    bool isPlaying();

    LionOpportunity *prevopp = nullptr;

    double getCoop(int nbpart, bool truecoop = false);

    void setCoop(int nbpart, double val);

    bool newopp{};


protected:
    std::vector<double> coopCache;


};


#endif //ROBOROBO3_LIONWORLDMODEL_H
