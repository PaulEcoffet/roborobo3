/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <Lion/include/LionWorldModel.h>
#include <Lion/include/LionWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "Lion/include/LionWorldObserver.h"
#include "Lion/include/LionController.h"
#include "Lion/include/LionSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>

using boost::algorithm::clamp;


LionWorldObserver::LionWorldObserver(World *__world) :
        WorldObserver(__world), robotsToTeleport(), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    LionSharedData::initSharedData();

    /********************/
    /* Coherence checks */
    /********************/
    if (LionSharedData::frictionCoef != 0)
    {
        assert(!LionSharedData::fixRobotNb);
    }

    // Assert that no object can be covered twice by the proximity teleport, proximityTeleport == 0 means no constraint
    assert(LionSharedData::proximityTeleport >= 0 &&
           LionSharedData::proximityTeleport <= (gNbOfPhysicalObjects / 2) - 1);
    assert(gNbOfPhysicalObjects % LionSharedData::nbCluster == 0);


    ///////////////////////////////////////////////////////////////////////////

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tind\trep\tfake\tfitness\n");

    std::vector<std::string> url;
    if (gRemote.empty())
    {
        std::cerr << "[WARNING] gRemote needs to be defined.";
        url.emplace_back("127.0.0.1");
        url.emplace_back("1703");
    }
    else
    {
        boost::split(url, gRemote, boost::is_any_of(":"));
    }
    pyevo.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_nbIndividuals = gInitialNumberOfRobots;
    gMaxIt = -1;

    /* build variability coef distribution */

    variabilityCoef.resize(m_nbIndividuals, 0);
    boost::math::normal normal(1, LionSharedData::fakeCoefStd);


    if (LionSharedData::fakeCoefMulSym)
    {
        double minquantile = boost::math::cdf(normal, 1. / LionSharedData::fakeCoef);
        double maxquantile = boost::math::cdf(normal, 1);
        double stepquantile = (maxquantile - minquantile) / ((m_nbIndividuals / 2) - 1);
        double curquantile = minquantile;
        for (int i = 0; i < m_nbIndividuals / 2; i++)
        {

            variabilityCoef[i] = boost::math::quantile(normal, curquantile);
            curquantile += stepquantile;
        }
        for (int i = 0; i < m_nbIndividuals / 2; i++)
        {
            variabilityCoef[m_nbIndividuals - i] = 1. / variabilityCoef[i];
        }
    }
    else
    {
        double minquantile = boost::math::cdf(normal, 1 - LionSharedData::fakeCoef);
        double maxquantile = boost::math::cdf(normal, 1 + LionSharedData::fakeCoef);
        double stepquantile = (maxquantile - minquantile) / (m_nbIndividuals - 1);
        double curquantile = minquantile;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            variabilityCoef[i] = boost::math::quantile(normal, curquantile);
            assert(variabilityCoef[i] <= 1 + LionSharedData::fakeCoef + 0.1);
            assert(variabilityCoef[i] >= 1 - LionSharedData::fakeCoef - 0.1);

            curquantile += stepquantile;
        }

    }


}


LionWorldObserver::~LionWorldObserver()
{
    delete m_fitnessLogManager;
};

void LionWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<LionController * >(m_world->getRobot(0)->getController())->getWeights().size();
    std::vector<double> minbounds(nbweights, -10);
    std::vector<double> maxbounds(nbweights, 10);
    std::vector<double> minguess(nbweights, -1);
    std::vector<double> maxguess(nbweights, 1);

    if (LionSharedData::fixCoop)
    {
        minbounds[0] = 0;
        maxbounds[0] = 1;
        minguess[0] = 0;
        maxguess[0] = 0.00001;
    }
    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights, minbounds, maxbounds, minguess, maxguess);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();
}


void LionWorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == LionSharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(i)->getWorldModel());
            m_fitnesses[i] += wm->_fitnessValue;
            m_curfitnesses[i] = wm->_fitnessValue;
        }
        logFitnesses(m_curfitnesses);
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;

        resetEnvironment();
    }
    if (m_curEvaluationInGeneration == LionSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
    }
}

void LionWorldObserver::stepPost()
{
    /* Plays */
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();

    /* Move */
    /* Teleport robots */
    if (LionSharedData::teleportRobots)
    {
        for (auto rid: robotsToTeleport)
        {
            m_world->getRobot(rid)->unregisterRobot();
            m_world->getRobot(rid)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                       gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
            m_world->getRobot(rid)->registerRobot();
        }
    }

    /*if (LionSharedData::tpToNewObj)
    {
        auto randomPhys = std::uniform_int_distribution<int>(0, gNbOfPhysicalObjects - 1);
        for (int i = 0; i < m_world->getNbOfRobots(); i++)
        {
            auto *rob = m_world->getRobot(i);
            if (dynamic_cast<LionWorldModel* >(rob->getWorldModel())->teleport) {
                int dest_obj = randomPhys(engine);
                double angle = (float) i / gNbOfRobots * 2 * M_PI;
                PhysicalObject *physobj = gPhysicalObjects[dest_obj];
                rob->unregisterRobot();
                rob->setCoord(physobj->getXCenterPixel()+ cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2),
                              physobj->getYCenterPixel()+ sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2));
                rob->setCoordReal(physobj->getXCenterPixel()+ cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2),
                                  physobj->getYCenterPixel()+ sin(angle) *(gPhysicalObjectDefaultRadius + gRobotWidth / 2));
                rob->getWorldModel()->_agentAbsoluteOrientation = 0;
                rob->registerRobot();
            }
        }
    }*/

    for (auto id: objectsToTeleport)
    {
        gPhysicalObjects[id]->unregisterObject();
        gPhysicalObjects[id]->resetLocation();
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();
    robotsToTeleport.clear();

    if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 0)
    {
        if (LionSharedData::takeVideo and m_curEvaluationInGeneration == 0)
        {
            saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
        }
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
        {
            if (m_logall.is_open())
            {
                m_logall.close();
            }
            m_logall.open(gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt");
            m_logall
                    << "eval\titer\tid\ta\tfakeCoef\tplaying\toppId\tnbOnOpp\tcurCoop\tmeanOwn\tmeanTotal\tpunish\tspite\n";
        }
        for (int i = 0; i < m_world->getNbOfRobots(); i++)
        {
            auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(i)->getWorldModel());
            double nbOnOpp = wm->nbOnOpp;
            if (LionSharedData::fixRobotNb && nbOnOpp > 2)
            {
                nbOnOpp = 2;
            }
            m_logall << m_curEvaluationInGeneration << "\t"
                     << m_curEvaluationIteration << "\t"
                     << i << "\t"
                     << wm->selfA << "\t"
                     << wm->fakeCoef << "\t"
                     << wm->isPlaying() << "\t"
                     << ((wm->opp != nullptr) ? wm->opp->getId() : -1) << "\t"
                     << nbOnOpp << "\t"
                     << wm->_cooperationLevel * wm->fakeCoef << "\t"
                     << wm->meanLastOwnInvest() << "\t"
                     << wm->meanLastTotalInvest() << "\t"
                     << wm->punishment << "\t"
                     << wm->spite
                     << "\n";
        }
    }
    else if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 1 && m_curEvaluationIteration == 0)
    {
        m_logall.flush(); // Let's flush now that we have written everything.
        m_logall.close();
    }

}


void LionWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    m_individuals = pyevo.getNextGeneration(m_individuals, m_fitnesses);
    if (m_individuals.empty())
    {
        exit(0);
    }
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> LionWorldObserver::getSortedFitnesses() const
{
    std::vector<std::pair<int, double>> fitnesses(m_individuals.size());
    for (int i = 0; i < m_individuals.size(); i++)
    {
        fitnesses[i].first = i;
        fitnesses[i].second = m_fitnesses[i];
    }
    std::sort(fitnesses.begin(), fitnesses.end(),
              [](std::pair<int, double> a, std::pair<int, double> b) { return a.second < b.second; });
    return fitnesses;
}

void LionWorldObserver::logFitnesses(const std::vector<double> &curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(i)->getWorldModel());
        out << m_generationCount << "\t"
            << i << "\t"
            << m_curEvaluationInGeneration << "\t"
            << cur_wm->fakeCoef << "\t"
            << curfitness[i] << "\n";
    }
    m_fitnessLogManager->write(out.str());
    m_fitnessLogManager->flush();
}

void LionWorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects)
    {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
        auto *rwm = dynamic_cast<LionWorldModel *>(robot->getWorldModel());
        rwm->lastCommonKnowledgeReputation.clear();
    }


    for (auto *object: gPhysicalObjects)
    {
        object->resetLocation();
        object->registerObject();
    }

    // Randomize the fakeList so that the last fakeRobots aren't necessarily the ones who cooperate the most.
    std::shuffle(variabilityCoef.begin(), variabilityCoef.end(), engine);

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        if (LionSharedData::tpToNewObj)
        {
            robot->getWorldModel()->_agentAbsoluteOrientation = 0;
        }
        auto *wm = dynamic_cast<LionWorldModel *>(robot->getWorldModel());
        if (LionSharedData::fakeRobots)
        {
            wm->fakeCoef = variabilityCoef[iRobot];
            assert(LionSharedData::fakeCoefMulSym || wm->fakeCoef <= 1 + LionSharedData::fakeCoef + 0.1);
            assert(LionSharedData::fakeCoefMulSym || wm->fakeCoef >= 1 - LionSharedData::fakeCoef - 0.1);
        }
        else
        {
            wm->fakeCoef = 1.0;
        }
        wm->setNewSelfA();
    }
}

void LionWorldObserver::computeOpportunityImpacts()
{
    const double b = LionSharedData::b;
    double sum_payoff = 0;
    int nb_payoffs = 0;

    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->opp = nullptr;
        wm->nbOnOpp = 0;
        wm->arrival = 0;
        wm->punishment = 0;
    }

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<LionOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (LionSharedData::fixRobotNb && n > 2)
        {
            n = 2;
        }

        int arrival = 1;
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(index)->getWorldModel());
            wm->onOpportunity = true;
            wm->opp = opp;
            wm->nbOnOpp = opp->getNbNearbyRobots();
            wm->arrival = arrival;
            arrival++;
        }

        // If we only give reward for the first two robots
        if (LionSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
        {
            itmax = opp->getNearbyRobotIndexes().begin() + 2;
        }

        for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
        {
            const auto *const wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(*index)->getWorldModel());
            double coop = clamp(wm->_cooperationLevel * wm->fakeCoef, 0, LionSharedData::maxCoop);
            if (LionSharedData::meanA == 0) // TODO SUPER UGLY, make parameter
            {
                coop = clamp(wm->_cooperationLevel + (wm->fakeCoef - 1) * 5, 0, LionSharedData::maxCoop);
            }
            totalInvest += coop;
            totalA += wm->selfA;
            for (auto oindex = opp->getNearbyRobotIndexes().begin(); oindex != itmax; oindex++)
            {
                if (oindex == index)
                { continue; }
                auto *owm = dynamic_cast<LionWorldModel *>(m_world->getRobot(*oindex)->getWorldModel());
                owm->updateOtherReputation(wm->_id, coop + randgaussian() * LionSharedData::reputationNoise);
            }
        }

        for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
        {
            auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(*index)->getWorldModel());
            double coop = clamp(wm->_cooperationLevel * wm->fakeCoef, 0, LionSharedData::maxCoop);
            if (LionSharedData::meanA == 0) // TODO SUPER UGLY, make parameter
            {
                coop = clamp(wm->_cooperationLevel + (wm->fakeCoef - 1) * 5, 0, LionSharedData::maxCoop);
            }
            wm->appendOwnInvest(coop);
            if (n >= 2)
            {
                wm->appendToCommonKnowledgeReputation(coop);
            }
            if (LionSharedData::onlyOtherInTotalInv)
            {
                wm->appendTotalInvest(totalInvest - coop);
            }
            else
            {
                wm->appendTotalInvest(totalInvest);
            }
            double curpayoff = payoff(coop, totalInvest, n, wm->selfA, b) /
                               (LionSharedData::evaluationTime - LionSharedData::fitnessUnlockedIter);
            if (m_curEvaluationIteration >= LionSharedData::fitnessUnlockedIter)
            {
                wm->_fitnessValue += curpayoff;
            }
            sum_payoff += curpayoff;
            nb_payoffs += 1;
        }

        if (LionSharedData::punishment)
        {
            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                const double spite = clamp(wm->spite, 0, LionSharedData::maxCoop);
                for (auto other = opp->getNearbyRobotIndexes().begin(); other != itmax; other++)
                {
                    auto *o_wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(*other)->getWorldModel());
                    if (other != index) // if it's a different robot
                    {
                        o_wm->punishment += spite;
                        o_wm->_fitnessValue -= spite * 3; // TODO Make punishment coef variable
                    }
                    else
                    {
                        wm->_fitnessValue -= spite; // cost of punishing someone
                    }
                }
            }
        }


        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;
    }

    // Give reward for all the lonely walkers
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<LionWorldModel *>(m_world->getRobot(i)->getWorldModel());
        if (!wm->isPlaying())
        {
            wm->_fitnessValue += std::min(LionSharedData::sigma, 0.8 * sum_payoff / nb_payoffs);
        }
    }
}

static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound) / (1 + exp(-slope * (x - inflexionPoint)));
}


double LionWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                 const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);
    if (LionSharedData::atLeastTwo and n < 2)
    {
        res = 0; // No payoff if you are alone
    }
    else
    {
        res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;
    }
    if (LionSharedData::frictionCoef > 0)
    {
        res *= (1 - sigmoid(n, 0, 1, LionSharedData::frictionCoef, LionSharedData::frictionInflexionPoint));
    }
    if (LionSharedData::temperature > 0)
    {
        res = std::exp(res / LionSharedData::temperature);
    }
    return res;
}


void LionWorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<LionOpportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void LionWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<LionController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void LionWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<LionController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void LionWorldObserver::addRobotToTeleport(int robotId)
{
    robotsToTeleport.insert(robotId);
}

void LionWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}
