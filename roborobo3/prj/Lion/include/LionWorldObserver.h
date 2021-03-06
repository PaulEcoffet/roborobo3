//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_LIONWORLDOBSERVER_H
#define ROBOROBO3_LIONWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "LionController.h"
#include "LionScoreLogger.h"
#include "LionWorldModel.h"

using json = nlohmann::json;

class LionWorldObserver : public WorldObserver
{
public:
    explicit LionWorldObserver(World *__world);

    ~LionWorldObserver() override;

    void reset() override;

    void logAgent(LionWorldModel *wm);

    void stepEvolution();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<double> &curfitness);

    void resetEnvironment();

    void stepPre() override;

    void stepPost() override;


    void addObjectToTeleport(int id);

    static double payoff(double invest, double totalInvest, int n, double a, double b);


    LionScoreLogger *getScoreLogger();

    bool isJustAfterLoggingTime() const;

    bool isLoggingTime() const;


    bool logScore();

protected:
    World *m_world;
    ogzstream m_fitnessLogManager;

    int m_curEvaluationInGeneration;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
    int m_generationCount;
    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;
    std::vector<double> m_curfitnesses;
    std::vector<int> m_curnbparticipation;
    std::vector<bool> m_curparticipationindexes;
    LionScoreLogger scorelogger;

    PyevoInterface pyevo;

    void clearRobotFitnesses();

    void loadGenomesInRobots(const std::vector<std::vector<double>> &genomes);

    std::set<int> objectsToTeleport;
    ogzstream m_logall;
    std::vector<double> variabilityCoef;


    void setWhichRobotsPlay();

    void cleanup();

    int m_trueCurEvaluationInGeneration;

    void printCoopStats(int i);
};


#endif //ROBOROBO3_LIONWORLDOBSERVER_H
