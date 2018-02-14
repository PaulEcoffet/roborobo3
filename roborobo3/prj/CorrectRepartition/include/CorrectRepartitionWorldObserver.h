//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_CORRECTREPARTITIONWORLDOBSERVER_H
#define ROBOROBO3_CORRECTREPARTITIONWORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include "core/Utilities/LogManager.h"
#include "contrib/json/json.hpp"
#include "CorrectRepartitionController.h"

using json = nlohmann::json;

class CorrectRepartitionWorldObserver : public WorldObserver
{
public:
    explicit CorrectRepartitionWorldObserver(World *__world);
    ~CorrectRepartitionWorldObserver() override;

    void stepPre() override;
    void stepPost() override;
    void reset() override;
    void stepEvolution();


    void resetEnvironment();

protected:
    World *m_world;
    LogManager *m_fitnessLogManager;
    LogManager* m_observer;
    json m_genomesLogJson;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
    int m_curEvaluationInGeneration;
    int m_curBatch;
    int m_generationCount;

    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;
    std::vector<int> m_shuffledIndividualId;

    PyevoInterface pycma;

    void computeOpportunityImpact();
    double payoff(const int nbRobots) const;
    void clearOpportunityNearbyRobots();
    void clearRobotFitnesses();
    void activateOnlyRobot(int batchIndex);
    void monitorPopulation() const;

    int m_batchSize = 0;
};


#endif //ROBOROBO3_CORRECTREPARTITIONWORLDOBSERVER_H
