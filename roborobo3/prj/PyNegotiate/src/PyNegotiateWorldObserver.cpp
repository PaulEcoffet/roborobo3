/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <PyNegotiate/include/PyNegotiateWorldModel.h>
#include <PyNegotiate/include/PyNegotiateWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "PyNegotiate/include/PyNegotiateWorldObserver.h"
#include "PyNegotiate/include/PyNegotiateController.h"
#include "PyNegotiate/include/PyNegotiateSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
#include <PyNegotiate/include/PyNegotiateAgentObserver.h>
//#include <cv.hpp>

using boost::algorithm::clamp;


PyNegotiateWorldObserver::PyNegotiateWorldObserver(World *__world) :
        WorldObserver(__world), robotsToTeleport(), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    PyNegotiateSharedData::initSharedData();

    /********************/
    /* Coherence checks */
    /********************/
    if (PyNegotiateSharedData::frictionCoef != 0)
    {
        assert(!PyNegotiateSharedData::fixRobotNb);
    }

    // Assert that no object can be covered twice by the proximity teleport, proximityTeleport == 0 means no constraint
    assert(PyNegotiateSharedData::proximityTeleport >= 0 &&
           PyNegotiateSharedData::proximityTeleport <= (gNbOfPhysicalObjects / 2) - 1);
    assert(gNbOfPhysicalObjects % PyNegotiateSharedData::nbCluster == 0);


    ///////////////////////////////////////////////////////////////////////////

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt.gz";
    m_fitnessLogManager.open(fitnessLogFilename.c_str());
    m_fitnessLogManager << "gen\tind\trep\tfake\tfitness\n";


    m_nbIndividuals = gInitialNumberOfRobots;
    gMaxIt = -1;

    /* build variability coef distribution */

    variabilityCoef.resize(m_nbIndividuals, 0);

    double startcoef = -PyNegotiateSharedData::fakeCoef;
    double endcoef = +PyNegotiateSharedData::fakeCoef;
    double stepcoef = (endcoef - startcoef) / (m_nbIndividuals - 1);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        variabilityCoef[i] = startcoef + i * stepcoef;
        assert(variabilityCoef[i] <= PyNegotiateSharedData::fakeCoef + 0.1);
        assert(variabilityCoef[i] >= -PyNegotiateSharedData::fakeCoef - 0.1);
    }

    std::pair<int, int> coord(-1, -1);
    bool run = true;
    int i = 0;
    while (run)
    {
        gProperties.checkAndGetPropertyValue("availableslot[" + std::to_string(i) + "].x", &coord.first, false);
        gProperties.checkAndGetPropertyValue("availableslot[" + std::to_string(i) + "].y", &coord.second, false);
        if (coord.first != -1)
        {
            availableslots.emplace(coord.first, coord.second);
            coord.first = -1;
            coord.second = -1;
            i++;
        }
        else
        {
            run = false;
        }
    }
}


PyNegotiateWorldObserver::~PyNegotiateWorldObserver()
{
    m_logall.close();
}

void PyNegotiateWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;
    resetEnvironment();
}


void PyNegotiateWorldObserver::stepPre()
{
    bool resetEnv = false;
    if (m_curEvaluationIteration == PyNegotiateSharedData::evaluationTime || endEvaluationNow)
    {
        m_curEvaluationIteration = 0;
        endEvaluationNow = false;
        std::vector<double> coop(m_nbIndividuals, 0);
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(i)->getWorldModel());
            m_fitnesses[i] += wm->_fitnessValue;
            m_curfitnesses[i] = wm->_fitnessValue;
            coop[i] = wm->getCoop(true);
        }
        logFitnesses(m_curfitnesses);
        std::sort(coop.begin(), coop.end());
        std::cout << "coop: " << coop[0] << ", " << coop[m_nbIndividuals / 4] << ", "
                  << coop[m_nbIndividuals / 2] << ", " << coop[3 * m_nbIndividuals / 4] << ", "
                  << coop[m_nbIndividuals - 1] << std::endl;
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;
        resetEnv = true;
    }
    if (m_curEvaluationInGeneration == PyNegotiateSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        m_generationCount++;
        if ((m_generationCount + 1) % PyNegotiateSharedData::logEveryXGen == 0)
        {
            if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
            {
                m_logall.close();
                m_logall.open((gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
                m_logall << "eval\titer\tid1\tfakeCoef1\ttrueCoop1\tid2\tfakeCoef2\ttrueCoop2\tAccept1\tAccept2"
                         << std::endl;
                logSeekTime();
            }
            if (PyNegotiateSharedData::takeVideo)
            {
                saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
            }
        }
        else if ((m_generationCount + 1) % PyNegotiateSharedData::logEveryXGen == 1 && m_curEvaluationIteration == 0)
        {
            m_logall.close();
        }
        if (resetEnv)
        {
            resetEnvironment();
        }
    }

    for (int i = 0; i < gInitialNumberOfRobots; i++)
    {
        auto *rob = m_world->getRobot(i);
        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(rob->getWorldModel());
        if (!wm->seeking)
        {
            if (!PyNegotiateSharedData::wander)
            {
                wm->_desiredTranslationalValue = 0;
                wm->_desiredRotationalVelocity = 0;
                wm->setAlive(false);
            }
            if (PyNegotiateSharedData::tau != 0 && random01() < 1.0 / PyNegotiateSharedData::tau)
            {
                wm->setAlive(true);
                wm->seeking = true;
                if (PyNegotiateSharedData::putOutOfGame)
                {
                    const auto new_pos = rob->findRandomLocation(gLocationFinderMaxNbOfTrials);
                    rob->setCoord(new_pos.first, new_pos.second);
                    rob->setCoordReal(new_pos.first, new_pos.second);
                }
            }
        }
    }
}

void PyNegotiateWorldObserver::stepPost()
{
    /* Plays */
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();

    /* Move */
    /* Teleport robots */
    if (PyNegotiateSharedData::teleportRobots)
    {
        for (auto rid: robotsToTeleport)
        {
            auto *rob = m_world->getRobot(rid);
            rob->unregisterRobot();
            const auto new_pos = rob->findRandomLocation(gLocationFinderMaxNbOfTrials);
            rob->setCoord(new_pos.first, new_pos.second);
            rob->setCoordReal(new_pos.first, new_pos.second);
            rob->registerRobot();
        }
    }

    std::set<int> toRetry;

    for (auto id: objectsToTeleport)
    {
        double prevx = gPhysicalObjects[id]->getXReal();
        double prevy = gPhysicalObjects[id]->getYReal();
        gPhysicalObjects[id]->unregisterObject();
        if (!PyNegotiateSharedData::randomObjectPositions)
        {
            gPhysicalObjects[id]->setCoordinates(curavailableslots.front().first, curavailableslots.front().second);
            curavailableslots.pop();
            curavailableslots.emplace(prevx, prevy);
        }
        else
        {
            int tries = gPhysicalObjects[id]->findRandomLocation();
            if (tries == gLocationFinderMaxNbOfTrials)
            {
                toRetry.emplace(id);
                gPhysicalObjects[id]->setCoordinates(prevx, prevy);
            }
        }
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();
    robotsToTeleport.clear();
    objectsToTeleport = toRetry;
    m_curEvaluationIteration++;
}


void PyNegotiateWorldObserver::stepEvolution()
{
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> PyNegotiateWorldObserver::getSortedFitnesses() const
{
    std::vector<std::pair<int, double>> fitnesses(m_individuals.size());
    for (size_t i = 0; i < m_individuals.size(); i++)
    {
        fitnesses[i].first = i;
        fitnesses[i].second = m_fitnesses[i];
    }
    std::sort(fitnesses.begin(), fitnesses.end(),
              [](std::pair<int, double> a, std::pair<int, double> b) { return a.second < b.second; });
    return fitnesses;
}

void PyNegotiateWorldObserver::logFitnesses(const std::vector<double> &curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(i)->getWorldModel());
        out << m_generationCount << "\t"
            << i << "\t"
            << m_curEvaluationInGeneration << "\t"
            << cur_wm->fakeCoef << "\t"
            << curfitness[i] << "\n";
    }
    m_fitnessLogManager << out.str();
    m_fitnessLogManager.flush();
}

void PyNegotiateWorldObserver::resetEnvironment()
{
    endEvaluationNow = false;
    curavailableslots = availableslots;

    nbOfRobotsWhoPlayed = 0;
    for (auto object: gPhysicalObjects)
    {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }


    for (auto *object: gPhysicalObjects)
    {
        object->resetLocation();
        if (PyNegotiateSharedData::randomObjectPositions)
        {
            object->findRandomLocation();
        }
        object->registerObject();
    }

    // Randomize the fakeList so that the last fakeRobots aren't necessarily the ones who cooperate the most.
    std::shuffle(variabilityCoef.begin(), variabilityCoef.end(), engine);

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        if (PyNegotiateSharedData::tpToNewObj)
        {
            robot->getWorldModel()->_agentAbsoluteOrientation = 0;
        }
        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(robot->getWorldModel());
        wm->setAlive(true);
        wm->seeking = true;
        if (PyNegotiateSharedData::fakeRobots)
        {
            wm->fakeCoef = variabilityCoef[iRobot];
        }
        else
        {
            wm->fakeCoef = 0;
        }
        wm->setNewSelfA();
    }
}

void PyNegotiateWorldObserver::computeOpportunityImpacts()
{
    double b = PyNegotiateSharedData::b;
    // Mark all robots as not on an cooperation opportunity
    mark_all_robots_as_alone();

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<PyNegotiateOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (n != 0)
        {
            if (PyNegotiateSharedData::fixRobotNb && n > 2)
            {
                n = 2;
            }

            mark_robots_on_opp(opp);

            // If we only give reward for the first two robots
            if (PyNegotiateSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
            {
                itmax = opp->getNearbyRobotIndexes().begin() + 2;
            }

            bool everyone_agree = true;
            std::vector<double> agentsCoop(n, 0);
            std::vector<double> agentsFake(n, 0);
            std::vector<bool> agentsAccept(n, false);
            int i = 0;

            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *const wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                agentsCoop[i] = wm->getCoop(true);
                agentsFake[i] = wm->fakeCoef;
                totalInvest += coop;
                totalA += wm->selfA;
                i++;
            }

            if (n > 1)
            {
                opp->kill();
                i = 0;
                for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++, i++)
                {
                    auto *const ctl = dynamic_cast<PyNegotiateController *>(m_world->getRobot(*index)->getController());
                    bool accept = ctl->acceptPlay();
                    agentsAccept[i] = accept;
                    everyone_agree = everyone_agree && accept;
                }
                if ((m_generationCount + 1) % PyNegotiateSharedData::logEveryXGen == 0)
                {
                    m_logall << m_curEvaluationInGeneration << "\t"
                             << m_curEvaluationIteration << "\t"
                             << opp->getNearbyRobotIndexes()[0] << "\t"
                             << agentsFake[0] << "\t"
                             << agentsCoop[0] << "\t"
                             << opp->getNearbyRobotIndexes()[1] << "\t"
                             << agentsFake[1] << "\t"
                             << agentsCoop[1] << "\t"
                             << agentsAccept[0] << "\t"
                             << agentsAccept[1] << std::endl;
                }
                if (everyone_agree)
                {
                    for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
                    {
                        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                        wm->seeking = false;
                        double coop = wm->getCoop();
                        double curpayoff = payoff(coop, totalInvest, n, wm->selfA, b) /
                                           PyNegotiateSharedData::nbEvaluationsPerGeneration;
                        if (PyNegotiateSharedData::tau != 0)
                        {
                            wm->_fitnessValue +=
                                    curpayoff * PyNegotiateSharedData::tau / PyNegotiateSharedData::evaluationTime;
                        }
                        else
                        {
                            wm->_fitnessValue += curpayoff;
                            nbOfRobotsWhoPlayed++;
                            if (nbOfRobotsWhoPlayed == m_nbIndividuals)
                            {
                                std::cout << "evaluation shorten, everyone has a payoff" << std::endl;
                                endEvaluationNow = true;
                            }
                        }
                        if (PyNegotiateSharedData::putOutOfGame)
                        {
                            m_world->getRobot(*index)->unregisterRobot();
                            m_world->getRobot(*index)->setCoord(2, 2);
                            m_world->getRobot(*index)->setCoordReal(2, 2);
                            m_world->getRobot(*index)->registerRobot();
                        }
                    }
                }
            }
        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;
    }
}

void PyNegotiateWorldObserver::mark_robots_on_opp(PyNegotiateOpportunity *opp) const
{
    int arrival = 1;
    for (auto index : opp->getNearbyRobotIndexes())
    {
        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(index)->getWorldModel());
        wm->onOpportunity = true;
        wm->opp = opp;
        wm->nbOnOpp = opp->getNbNearbyRobots();
        wm->arrival = arrival;
        arrival++;
    }
}

void PyNegotiateWorldObserver::mark_all_robots_as_alone() const
{
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->opp = nullptr;
        wm->nbOnOpp = 0;
        wm->arrival = 0;
        wm->punishment = 0;
    }
}

static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound) / (1 + exp(-slope * (x - inflexionPoint)));
}


double PyNegotiateWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                        const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);
    if (PyNegotiateSharedData::atLeastTwo and n < 2)
    {
        res = 0; // No payoff if you are alone
    }
    else
    {
        res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;
    }
    if (PyNegotiateSharedData::frictionCoef > 0)
    {
        res *= (1 -
                sigmoid(n, 0, 1, PyNegotiateSharedData::frictionCoef, PyNegotiateSharedData::frictionInflexionPoint));
    }
    if (PyNegotiateSharedData::temperature > 0)
    {
        res = std::exp(res / PyNegotiateSharedData::temperature);
    }
    return res;
}


void PyNegotiateWorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PyNegotiateOpportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void PyNegotiateWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<PyNegotiateController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void PyNegotiateWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
}

void PyNegotiateWorldObserver::addRobotToTeleport(int robotId)
{
    robotsToTeleport.insert(robotId);
}

void PyNegotiateWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}

void PyNegotiateWorldObserver::logSeekTime() const
{
    std::cout << "Saving seek time" << std::endl;
    std::ofstream f_seektime(
            (gLogDirectoryname + "/seektime_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
    f_seektime << "id\tseekCount\n";
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *rob = gWorld->getRobot(i);
        auto *obs = dynamic_cast<PyNegotiateAgentObserver *>(rob->getObserver());
        f_seektime << i << "\t" << obs->getSeekTime() << "\n";
    }
    f_seektime.close();
}
