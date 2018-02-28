//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "PartnerControlController.h"

using json = nlohmann::json;

class PartnerControlWorldObserver : public WorldObserver
{
public:
    explicit PartnerControlWorldObserver(World *__world);
    ~PartnerControlWorldObserver() override;

    void reset() override;
    void stepEvolution();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void resetEnvironment();
    void stepPre() override;


protected:
    World *m_world;
    LogManager *m_fitnessLogManager;
    LogManager* m_observer;

    int m_curEvaluationInGeneration;
    int m_curInd;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
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


#endif //ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H
