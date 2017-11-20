//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H
#define ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include "core/Utilities/LogManager.h"
#include "contrib/json/json.hpp"
#include "PartnerControlController.h"

using json = nlohmann::json;

class PartnerControlWorldObserver : public WorldObserver
{
public:
    typedef struct {
        double fitness;
        PartnerControlController::genome genome;
    } individual;

    explicit PartnerControlWorldObserver(World *__world);
    ~PartnerControlWorldObserver() override;

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
    LogManager* m_observer;
    json m_genomesLogJson;

    int m_curEvalutionIteration;
    int _generationCount;

    std::mt19937 m_mt; // TODO: MUST BE THE SAME FOR THE WHOLE PROGRAM

    void initOpportunities();
    void computeOpportunityImpact();

    double payoff(const double invest, const double totalInvest) const;

    void clearOpportunityNearbyRobots();

    void clearRobotFitnesses();

    int m_curEvaluationInGeneration;
    int m_curInd;

    std::vector<individual> m_individuals;

    void activateOnlyRobot(int robotIndex);

    void monitorPopulation() const;

    int m_nbIndividuals;
};


#endif //ROBOROBO3_PARTNERCONTROLWORLDOBSERVER_H
