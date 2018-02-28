//
// Created by paul on 06/11/17.
//

#ifndef ROBOROBO3_CORRECTREPARTITIONANALYSISWORLDOBSERVER_H
#define ROBOROBO3_CORRECTREPARTITIONANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <contrib/json/json.hpp>
#include "World/World.h"
#include "Agents/Robot.h"


using json = nlohmann::json;

class CorrectRepartitionAnalysisWorldObserver : public WorldObserver
{
public:
    explicit CorrectRepartitionAnalysisWorldObserver(World *__world);

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


#endif //ROBOROBO3_CORRECTREPARTITIONANALYSISWORLDOBSERVER_H
