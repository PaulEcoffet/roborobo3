//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2WORLDOBSERVER_H
#define ROBOROBO3_COOPFIXED2WORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include <contrib/network/PyevoInterface.h>
#include "core/Utilities/LogManager.h"
#include "contrib/json/json.hpp"
#include "CoopFixed2Controller.h"

using json = nlohmann::json;

class CoopFixed2WorldObserver : public WorldObserver
{
public:
    explicit CoopFixed2WorldObserver(World *__world);
    ~CoopFixed2WorldObserver() override;

    void reset() override;
    void stepEvolution();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void resetEnvironment();
    void stepPre() override;
    void stepPost() override;


    void addRobotToTeleport(int robotId);

protected:
    World *m_world;
    LogManager *m_fitnessLogManager;

    int m_curEvaluationInGeneration;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
    int m_generationCount;
    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;
    PyevoInterface pyevo;


    void computeOpportunityImpacts();
    double payoff(double invest, double totalInvest, double cval=0.5) const;
    void registerRobotsOnOpportunities();
    void clearRobotFitnesses();
    void loadGenomesInRobots(const std::vector<std::vector<double>>& genomes);

    std::set<int> robotsToTeleport;
    int m_nbFakeRobots;
};


#endif //ROBOROBO3_COOPFIXED2WORLDOBSERVER_H
