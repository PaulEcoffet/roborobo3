/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_SKILLEDANALYSISOPPORTUNITY_H
#define ROBOROBO3_SKILLEDANALYSISOPPORTUNITY_H


#include "World/RoundObject.h"
#include "SkilledOpportunity.h"
#include "RoboroboMain/roborobo.h"
#include <set>

class SkilledAnalysisOpportunity : public SkilledOpportunity
{
public:
    explicit SkilledAnalysisOpportunity(int __id);

    void step() override;

    void setCoopValue(double coop);

    double getCoop() const;

    void setNbFakeRobots(int nbrobots);

    int getNbFakeRobots();

    void placeFakeRobot();

    void updateColor() override;


    std::vector<Robot *> fakerobots = {};

protected:
    double m_coop;
    int nbFakeRobots;

};


#endif //ROBOROBO3_SKILLEDANALYSISOPPORTUNITY_H
