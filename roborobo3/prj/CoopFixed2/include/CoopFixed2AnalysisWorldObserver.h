/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H
#define ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <contrib/json/json.hpp>
#include "core/World/World.h"
#include "core/Agents/Robot.h"

using json = nlohmann::json;


class CoopFixed2AnalysisWorldObserver : public WorldObserver
{
public:
    explicit CoopFixed2AnalysisWorldObserver(World *__world);

    void stepPre() override;
    void stepPost() override;
    void reset() override;


protected:
    double m_curCoop;
    json m_genomesJson;

    int m_curIterationInRep;
    int m_curRep;
    int m_curInd;


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

};


#endif //ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H