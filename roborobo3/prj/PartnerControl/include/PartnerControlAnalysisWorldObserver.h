//
// Created by paul on 06/11/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLANALYSISWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCONTROLANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <json/json.hpp>
#include "World/World.h"
#include "Agents/Robot.h"


using json = nlohmann::json;

class PartnerControlAnalysisWorldObserver : public WorldObserver
{
public:
    explicit PartnerControlAnalysisWorldObserver(World *__world);

    void stepPre() override;

    void reset() override;


protected:
    double m_curCoop;
    json m_genomesJson;

    int m_curIterationInRep;
    int m_curRep;

    std::ofstream m_log;

    void monitorPopulation();

    void resetEnvironment();

    json::iterator m_genomesIt;

    void loadGenome(const std::vector<double> &weights);

    int m_nbIterationPerRep;
    int m_nbRep;
    double m_stepCoop;

    void setAllOpportunitiesCoop(double coop);

    void computeOpportunityImpact();

    void clearOpportunityNearbyRobots();

    int m_curInd;
};


#endif //ROBOROBO3_PARTNERCONTROLANALYSISWORLDOBSERVER_H
