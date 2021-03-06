//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_SKILLEDTOURNAMENTWORLDOBSERVER_H
#define ROBOROBO3_SKILLEDTOURNAMENTWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "SkilledController.h"

using json = nlohmann::json;

class SkilledTournamentWorldObserver : public WorldObserver
{
public:
    explicit SkilledTournamentWorldObserver(World *__world);

    ~SkilledTournamentWorldObserver() override;

    void reset() override;


    void logAgent(SkilledWorldModel *wm);


    void stepEvolution();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<double> &curfitness);

    void resetEnvironment();

    void stepPre() override;

    void stepPost() override;


    void addObjectToTeleport(int id);

    static double payoff(double invest, double totalInvest, int n, double a, double b);


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

    void loadGenomesInRobots(const std::vector<std::vector<double>> &genomes);

    std::set<int> objectsToTeleport;
    std::ofstream m_logall;
    std::vector<double> variabilityCoef;

};


#endif //ROBOROBO3_SKILLEDTOURNAMENTWORLDOBSERVER_H
