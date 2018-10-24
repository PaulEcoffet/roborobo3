/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H
#define ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H


#include "World/RoundObject.h"
#include "CoopFixed2Opportunity.h"
#include "RoboroboMain/roborobo.h"
#include <set>

class CoopFixed2AnalysisOpportunity : public CoopFixed2Opportunity
{
public:
    explicit CoopFixed2AnalysisOpportunity(int __id);

    void step() override;

    void setCoopValue(double coop);

    void clearNearbyRobotIndexes();

    std::string inspect(std::string prefix) override;

    const std::vector<int> &getNearbyRobotIndexes() const override;

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const override;

    double getCoop() const;

    void setNbFakeRobots(int nbrobots);

    int getNbFakeRobots();

    void isWalked(int id) override;

    void placeFakeRobot();


    std::shared_ptr<Robot> fakerobot = nullptr;
protected:
    double m_coop;
    std::vector<int> nearbyRobotIndexes;

    void updateColor();

    int nbFakeRobots;

};


#endif //ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H
