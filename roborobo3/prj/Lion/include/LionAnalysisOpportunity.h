/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_LIONANALYSISOPPORTUNITY_H
#define ROBOROBO3_LIONANALYSISOPPORTUNITY_H


#include "World/RoundObject.h"
#include "LionOpportunity.h"
#include "RoboroboMain/roborobo.h"
#include <set>

class LionAnalysisOpportunity : public LionOpportunity
{
public:
    explicit LionAnalysisOpportunity(int __id);

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


#endif //ROBOROBO3_LIONANALYSISOPPORTUNITY_H
