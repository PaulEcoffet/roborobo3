//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_SKILLEDWORLDMODEL_H
#define ROBOROBO3_SKILLEDWORLDMODEL_H


#include <WorldModels/RobotWorldModel.h>
#include <deque>
#include "SkilledOpportunity.h"

class SkilledOpportunity;

class SkilledWorldModel : public RobotWorldModel {
public:
    SkilledWorldModel();

    double nbOnOpp = 0;
    bool onOpportunity;
    double selfA;

    void setNewSelfA();

    int arrival;
    SkilledOpportunity *opp = nullptr;

    bool fake;
    bool ingame = false;
    double fakeCoef;

    void reset();

    int teleport = -1;

    bool isPlaying();

    SkilledOpportunity *prevopp = nullptr;

    double getCoop(int nbpart, bool truecoop = false);

    void setCoopAlone(double val);

    void setCoopPartners(double val);

    void setCoops(double _coopalone, double _cooppartner);

    bool newopp{};


protected:
    double coopalone = 0;
    double cooppartner = 0;
    double skill = 0;
public:
    double getSkill() const;

    void setSkill(double skill);


};


#endif //ROBOROBO3_SKILLEDWORLDMODEL_H
