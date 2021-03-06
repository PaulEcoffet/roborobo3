/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H
#define ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <json/json.hpp>
#include <set>
#include "World/World.h"
#include "Agents/Robot.h"

using json = nlohmann::json;


class CoopFixed2AnalysisWorldObserver : public WorldObserver
{
public:
    explicit CoopFixed2AnalysisWorldObserver(World *__world);

    void stepPre() override;

    void stepPost() override;

    void reset() override;

    void registerRobotsOnOpportunities();

    void addObjectToTeleport(int id);


protected:
    double m_curCoop;
    int m_curnbrob;
    int m_maxrobnb;
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

    void computeOpportunityImpact();


    std::set<int> objectsToTeleport;

    void initObjects() const;
};


#endif //ROBOROBO3_COOPFIXED2ANALYSISWORLDOBSERVER_H
