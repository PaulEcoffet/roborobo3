//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_NEGOCIATEWORLDOBSERVER_H
#define ROBOROBO3_NEGOCIATEWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include <gzstream.h>
#include <queue>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "NegociateController.h"
/*
#include <opencv2/core.hpp>  // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // VideoWriter
#include <opencv2/imgproc/imgproc.hpp>  // channel manipulation
*/
using json = nlohmann::json;

class NegociateWorldObserver : public WorldObserver
{
public:
    explicit NegociateWorldObserver(World *__world);

    ~NegociateWorldObserver() override;

    void reset() override;

    void stepEvolution();

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<double> &curfitness);

    void resetEnvironment();

    void stepPre() override;

    void stepPost() override;


    void addRobotToTeleport(int robotId);

    void addObjectToTeleport(int id);

    static double payoff(double invest, double totalInvest, int n, double a, double b);


protected:
    World *m_world;
    ogzstream m_fitnessLogManager;

    //cv::VideoWriter outvid;

    int m_curEvaluationInGeneration;
    int m_curEvaluationIteration;
    int m_nbIndividuals;
    int m_generationCount;
    std::vector<std::vector<double>> m_individuals;
    std::vector<double> m_fitnesses;
    std::vector<double> m_curfitnesses;
    PyevoInterface pyevo;


    virtual void computeOpportunityImpacts();

    void registerRobotsOnOpportunities();

    void clearRobotFitnesses();

    void loadGenomesInRobots(const std::vector<std::vector<double>> &genomes);

    std::set<int> robotsToTeleport;
    std::set<int> objectsToTeleport;
    ogzstream m_logall;


    std::vector<double> variabilityCoef;

    void reward_lonely(double sum_payoff, int nb_payoffs) const;

    void mark_all_robots_as_alone() const;

    void mark_robots_on_opp(NegociateOpportunity *opp) const;

    bool endEvaluationNow = false;
    int nbOfRobotsWhoPlayed = 0;
    double emptyX = 122;
    double emptyY = 38;
    std::queue<std::pair<int, int>> availableslots;
    std::queue<std::pair<int, int>> curavailableslots;

    void logSeekTime();
};


#endif //ROBOROBO3_NEGOCIATEWORLDOBSERVER_H
