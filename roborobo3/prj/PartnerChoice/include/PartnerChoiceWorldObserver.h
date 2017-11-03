//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include "core/Utilities/LogManager.h"

class PartnerChoiceWorldObserver : public WorldObserver
{
public:
    explicit PartnerChoiceWorldObserver(World *__world);
    ~PartnerChoiceWorldObserver() override;

    void step() override;
    void reset() override;
    void stepEvaluation();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void logGenomes(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void createNextGeneration(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void resetEnvironment();

protected:
    World *m_world;
    LogManager *m_fitnessLogManager;
    LogManager *m_genomeLogManager;

    int m_curEvalutionIteration;
    int _generationCount;

    void initOpportunities();
    void computeOpportunityImpact();

    double payoff(const double invest, const double totalInvest) const;

    void initSharedData();

    void clearOpportunityNearbyRobots();

    void clearRobotFitnesses();

    int m_curEvaluationInGeneration;
};


#endif //ROBOROBO3_PARTNERCHOICEWORLDOBSERVER_H
