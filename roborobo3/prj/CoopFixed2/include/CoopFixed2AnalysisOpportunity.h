/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H
#define ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H


#include "World/RoundObject.h"
#include "CoopFixed2Opportunity.h"
#include <set>

class CoopFixed2AnalysisOpportunity: public CoopFixed2Opportunity
{
public:
    explicit CoopFixed2AnalysisOpportunity(int __id);
    void step() override;
    void setCoopValue(double coop);
    const std::set<int> & getNearbyRobotIndexes() const;
    void clearNearbyRobotIndexes();

    void isPushed(int id, std::tuple<double, double> speed) override;

    int getNbNearbyRobots() const override;
    double getCoop() const;

    protected:
    double m_coop;
    std::set<int> nearbyRobotIndexes;

    void updateColor();

    int m_nbprev;
};



#endif //ROBOROBO3_COOPFIXED2ANALYSISOPPORTUNITY_H
