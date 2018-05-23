//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_COOPFIXED2WORLDOBSERVER_H
#define ROBOROBO3_COOPFIXED2WORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
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

    void logFitnesses(const std::vector<double>& curfitness);
    void resetEnvironment();
    void stepPre() override;
    void stepPost() override;


    void addRobotToTeleport(int robotId);

    void addObjectToTeleport(int id);
    static double payoff(double invest, double totalInvest, int n, double a, double b, double d);


protected:
    World *m_world;
    LogManager *m_fitnessLogManager;

    int m_curEvaluationInGeneration;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
    int m_generationCount;
    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;
    std::vector<double> m_curfitnesses;
    PyevoInterface pyevo;


    void computeOpportunityImpacts();
    void registerRobotsOnOpportunities();
    void clearRobotFitnesses();
    void loadGenomesInRobots(const std::vector<std::vector<double>>& genomes);

    std::set<int> robotsToTeleport;
    int m_nbFakeRobots;
    std::set<int> objectsToTeleport;
    std::ofstream m_logall;
    std::vector<int> m_fakerobotslist;
    bool m_swapfake;
};


#endif //ROBOROBO3_COOPFIXED2WORLDOBSERVER_H
