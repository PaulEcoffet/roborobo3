//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "PartnerChoiceController.h"

using json = nlohmann::json;

class PartnerChoiceWorldObserver : public WorldObserver
{
public:
    explicit PartnerChoiceWorldObserver(World *__world);
    ~PartnerChoiceWorldObserver() override;

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
    int m_curInd;
    int m_generationCount;

    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;

    PyevoInterface pycma;

    void initOpportunities();
    void computeOpportunityImpact();
    double payoff(const double invest, const double totalInvest) const;
    void clearOpportunityNearbyRobots();
    void clearRobotFitnesses();
    void activateOnlyRobot(int robotIndex);
    void monitorPopulation() const;

};


#endif //ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H
