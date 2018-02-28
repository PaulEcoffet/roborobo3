//
// Created by paul on 06/11/17.
//

#ifndef ROBOROBO3_PARTNERCHOICEANALYSISWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCHOICEANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <json/json.hpp>
#include "World/World.h"
#include "Agents/Robot.h"


using json = nlohmann::json;

class PartnerChoiceAnalysisWorldObserver : public WorldObserver
{
public:
    explicit PartnerChoiceAnalysisWorldObserver(World *__world);

    void stepPre() override;
    void reset() override;


protected:
    double m_curCoop;
    json m_genomesJson;
    std::ofstream m_log;

    int m_curIterationInRep;
    int m_curRep;
    int m_onOppTotal = 0;

    void monitorPopulation();

    void resetEnvironment();

    json::iterator m_genomesIt;

    void loadGenome(const std::vector<double> &weights);
    void computeOpportunityImpact();
    void clearOpportunityNearbyRobots();

    int m_nbIterationPerRep;
    int m_nbRep;
    double m_stepCoop;

    void setAllOpportunitiesCoop(double coop);

    void logRep();

    int m_curInd;
};


#endif //ROBOROBO3_PARTNERCHOICEANALYSISWORLDOBSERVER_H
