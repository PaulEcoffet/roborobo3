/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <NegociateGym/include/NegociateGymWorldModel.h>
#include <NegociateGym/include/NegociateGymWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "NegociateGym/include/NegociateGymWorldObserver.h"
#include "NegociateGym/include/NegociateGymController.h"
#include "NegociateGym/include/NegociateGymSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
#include <NegociateGym/include/NegociateGymAgentObserver.h>
//#include <cv.hpp>

using boost::algorithm::clamp;


NegociateGymWorldObserver::NegociateGymWorldObserver(World *__world) :
        WorldObserver(__world), robotsToTeleport(), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    NegociateGymSharedData::initSharedData();

    /********************/
    /* Coherence checks */
    /********************/
    if (NegociateGymSharedData::frictionCoef != 0)
    {
        assert(!NegociateGymSharedData::fixRobotNb);
    }

    // Assert that no object can be covered twice by the proximity teleport, proximityTeleport == 0 means no constraint
    assert(NegociateGymSharedData::proximityTeleport >= 0 &&
           NegociateGymSharedData::proximityTeleport <= (gNbOfPhysicalObjects / 2) - 1);
    assert(gNbOfPhysicalObjects % NegociateGymSharedData::nbCluster == 0);


    ///////////////////////////////////////////////////////////////////////////

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt.gz";
    m_fitnessLogManager.open(fitnessLogFilename.c_str());
    m_fitnessLogManager << "gen\tind\trep\tfake\tfitness\n";

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
    pyroborobo.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_nbIndividuals = gInitialNumberOfRobots;
    gMaxIt = -1;

    /* build variability coef distribution */

    variabilityCoef.resize(m_nbIndividuals, 0);

    double startcoef = -NegociateGymSharedData::fakeCoef;
    double endcoef = +NegociateGymSharedData::fakeCoef;
    double stepcoef = (endcoef - startcoef) / (m_nbIndividuals - 1);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        variabilityCoef[i] = startcoef + i * stepcoef;
        assert(variabilityCoef[i] <= NegociateGymSharedData::fakeCoef + 0.1);
        assert(variabilityCoef[i] >= -NegociateGymSharedData::fakeCoef - 0.1);
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


NegociateGymWorldObserver::~NegociateGymWorldObserver()
{
    m_logall.close();
}

void NegociateGymWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;
    // Init fitness
    clearRobotFitnesses();

    m_individuals = pyroborobo.initRoborobo(m_nbIndividuals, );
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();
}


void NegociateGymWorldObserver::stepPre()
{
    bool resetEnv = false;
    if (m_curEvaluationIteration == NegociateGymSharedData::evaluationTime || endEvaluationNow)
    {
        m_curEvaluationIteration = 0;
        endEvaluationNow = false;
        std::vector<double> coop(m_nbIndividuals, 0);
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(i)->getWorldModel());
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
    if (m_curEvaluationInGeneration == NegociateGymSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
        if ((m_generationCount + 1) % NegociateGymSharedData::logEveryXGen == 0)
        {
            if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
            {
                std::cout << "**********************************\n";
                std::cout << "TIME TO LOG\n";
                std::cout << "**********************************\n";
                if (NegociateGymSharedData::takeVideo)
                {
                    /*
                    std::cout <<  "VIDEO\n";
                    outvid.release();
                    getSnapshot(gSnapshot);
                    cv::Size S(gSnapshot->w, gSnapshot->h);
                    outvid.open(gLogDirectoryname + "/mov_" + std::to_string(m_generationCount) + ".mp4",
                                cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60, S, true);
                    */
                }

                m_logall.close();
                m_logall.open((gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
                m_logall << "eval\titer\tid1\tfakeCoef1\ttrueCoop1\tid2\tfakeCoef2\ttrueCoop2\tAccept1\tAccept2"
                         << std::endl;
                logSeekTime();
            }
            if (NegociateGymSharedData::takeVideo)
            {
                saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
                /*
                    getSnapshot(gSnapshot);
                    SDL_Surface* bgrsurf = SDL_ConvertSurfaceFormat(gSnapshot, SDL_PIXELFORMAT_BGRA8888, 0);
                    saveImage(bgrsurf, "test");
                    cv::Mat img(gSnapshot->w, gSnapshot->h, CV_8UC4, bgrsurf->pixels);
                    cv::Mat imgNoAlpha;
                    cv::Mat mp4img;  // MP4 needs Blue Green Red (BGR) channels for the mp4 file
                    cv::cvtColor(img, imgNoAlpha, CV_BGRA2BGR);

        //            cv::cvtColor(imgNoAlpha, mp4img,CV_RGB2YUV_I420);

                    outvid << imgNoAlpha; // append the frame to the video (should be YUV but do not work)
                    SDL_FreeSurface(bgrsurf);
                    */
            }
        }
        else if ((m_generationCount + 1) % NegociateGymSharedData::logEveryXGen == 1 && m_curEvaluationIteration == 0)
        {
            m_logall.close();
            //outvid.release();
        }
        if (resetEnv)
        {
            resetEnvironment();
        }
    }

    for (int i = 0; i < gInitialNumberOfRobots; i++)
    {
        auto *rob = m_world->getRobot(i);
        auto *wm = dynamic_cast<NegociateGymWorldModel *>(rob->getWorldModel());
        if (!wm->seeking)
        {
            if (!NegociateGymSharedData::wander)
            {
                wm->_desiredTranslationalValue = 0;
                wm->_desiredRotationalVelocity = 0;
                wm->setAlive(false);
            }
            if (NegociateGymSharedData::tau != 0 && random01() < 1.0 / NegociateGymSharedData::tau)
            {
                wm->setAlive(true);
                wm->seeking = true;
                if (NegociateGymSharedData::putOutOfGame)
                {
                    rob->findRandomLocation(gAgentsInitAreaX,
                                            gAgentsInitAreaX + gAgentsInitAreaWidth,
                                            gAgentsInitAreaY,
                                            gAgentsInitAreaY + gAgentsInitAreaHeight);
                }
            }
        }
    }
}

void NegociateGymWorldObserver::stepPost()
{
    /* Plays */
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();

    /* Move */
    /* Teleport robots */
    if (NegociateGymSharedData::teleportRobots)
    {
        for (auto rid: robotsToTeleport)
        {
            m_world->getRobot(rid)->unregisterRobot();
            m_world->getRobot(rid)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                       gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
            m_world->getRobot(rid)->registerRobot();
        }
    }

    std::set<int> toRetry;

    for (auto id: objectsToTeleport)
    {
        double prevx = gPhysicalObjects[id]->getXReal();
        double prevy = gPhysicalObjects[id]->getYReal();
        gPhysicalObjects[id]->unregisterObject();
        if (!NegociateGymSharedData::randomObjectPositions)
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


void NegociateGymWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % NegociateGymSharedData::logEveryXGen == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    m_individuals = pyroborobo.getNextGeneration(m_individuals, m_fitnesses);
    if (m_individuals.empty())
    {
        exit(0);
    }
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> NegociateGymWorldObserver::getSortedFitnesses() const
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

void NegociateGymWorldObserver::logFitnesses(const std::vector<double> &curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(i)->getWorldModel());
        out << m_generationCount << "\t"
            << i << "\t"
            << m_curEvaluationInGeneration << "\t"
            << cur_wm->fakeCoef << "\t"
            << curfitness[i] << "\n";
    }
    m_fitnessLogManager << out.str();
    m_fitnessLogManager.flush();
}

void NegociateGymWorldObserver::resetEnvironment()
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
        auto *rwm = dynamic_cast<NegociateGymWorldModel *>(robot->getWorldModel());
        rwm->lastCommonKnowledgeReputation.clear();
    }


    for (auto *object: gPhysicalObjects)
    {
        object->resetLocation();
        if (NegociateGymSharedData::randomObjectPositions)
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
        if (NegociateGymSharedData::tpToNewObj)
        {
            robot->getWorldModel()->_agentAbsoluteOrientation = 0;
        }
        auto *wm = dynamic_cast<NegociateGymWorldModel *>(robot->getWorldModel());
        wm->setAlive(true);
        wm->seeking = true;
        if (NegociateGymSharedData::fakeRobots)
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

void NegociateGymWorldObserver::computeOpportunityImpacts()
{
    const double b = NegociateGymSharedData::b;
    // Mark all robots as not on an cooperation opportunity
    mark_all_robots_as_alone();

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<NegociateGymOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (n != 0)
        {
            if (NegociateGymSharedData::fixRobotNb && n > 2)
            {
                n = 2;
            }

            mark_robots_on_opp(opp);

            // If we only give reward for the first two robots
            if (NegociateGymSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
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
                auto *const wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                agentsCoop[i] = wm->getCoop(true);
                agentsFake[i] = wm->fakeCoef;
                totalInvest += coop;
                totalA += wm->selfA;
                i++;
            }


            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                wm->appendOwnInvest(coop);

                if (NegociateGymSharedData::onlyOtherInTotalInv)
                {
                    wm->appendTotalInvest(totalInvest - coop);
                }
                else
                {
                    wm->appendTotalInvest(totalInvest);
                }
            }

            i = 0;
            if (n > 1)
            {
                for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
                {
                    auto *const ctl = dynamic_cast<NegociateGymController *>(m_world->getRobot(*index)->getController());
                    bool accept = ctl->acceptPlay();
                    agentsAccept[i] = accept;
                    everyone_agree = everyone_agree && accept;
                    i++;
                }
            }
            if ((m_generationCount + 1) % NegociateGymSharedData::logEveryXGen == 0
                && n > 1)
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
            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                if (n > 1)
                {
                    opp->kill();

                    auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                    double coop = wm->getCoop();
                    if (everyone_agree)
                    {
                        double curpayoff = payoff(coop, totalInvest, n, wm->selfA, b) /
                                           NegociateGymSharedData::nbEvaluationsPerGeneration;
                        if (NegociateGymSharedData::doNotKill)
                        {
                            if (wm->_fitnessValue == 0 &&
                                curpayoff != 0) // curpayoff is VERY unlikely to be exactly 0 but prevent bugs
                            {
                                wm->_fitnessValue = curpayoff;
                                nbOfRobotsWhoPlayed++;
                                if (nbOfRobotsWhoPlayed == m_nbIndividuals and NegociateGymSharedData::tau == 0)
                                {
                                    std::cout << "evaluation shorten, everyone has a payoff" << std::endl;
                                    endEvaluationNow = true;
                                }
                            }
                        }
                        else
                        {
                            if (NegociateGymSharedData::tau != 0)
                            {
                                wm->_fitnessValue +=
                                        curpayoff * NegociateGymSharedData::tau / NegociateGymSharedData::evaluationTime;
                            }
                            else
                            {
                                wm->_fitnessValue += curpayoff;
                            }
                            wm->seeking = false;
                            nbOfRobotsWhoPlayed++;
                            if (nbOfRobotsWhoPlayed == m_nbIndividuals and NegociateGymSharedData::tau == 0)
                            {
                                std::cout << "evaluation shorten, everyone has a payoff" << std::endl;
                                endEvaluationNow = true;
                            }
                            if (NegociateGymSharedData::putOutOfGame)
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
        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;
    }
}

void NegociateGymWorldObserver::mark_robots_on_opp(NegociateGymOpportunity *opp) const
{
    int arrival = 1;
    for (auto index : opp->getNearbyRobotIndexes())
    {
        auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(index)->getWorldModel());
        wm->onOpportunity = true;
        wm->opp = opp;
        wm->nbOnOpp = opp->getNbNearbyRobots();
        wm->arrival = arrival;
        arrival++;
    }
}

void NegociateGymWorldObserver::mark_all_robots_as_alone() const
{
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->opp = nullptr;
        wm->nbOnOpp = 0;
        wm->arrival = 0;
        wm->punishment = 0;
    }
}

void NegociateGymWorldObserver::reward_lonely(double sum_payoff, int nb_payoffs) const
{
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(i)->getWorldModel());
        if (!wm->isPlaying())
        {
            wm->_fitnessValue += std::min(NegociateGymSharedData::sigma, 0.8 * sum_payoff / nb_payoffs);
        }
    }
}

static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound) / (1 + exp(-slope * (x - inflexionPoint)));
}


double NegociateGymWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                      const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);
    if (NegociateGymSharedData::atLeastTwo and n < 2)
    {
        res = 0; // No payoff if you are alone
    }
    else
    {
        res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;
    }
    if (NegociateGymSharedData::frictionCoef > 0)
    {
        res *= (1 - sigmoid(n, 0, 1, NegociateGymSharedData::frictionCoef, NegociateGymSharedData::frictionInflexionPoint));
    }
    if (NegociateGymSharedData::temperature > 0)
    {
        res = std::exp(res / NegociateGymSharedData::temperature);
    }
    return res;
}


void NegociateGymWorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<NegociateGymOpportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void NegociateGymWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<NegociateGymController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void NegociateGymWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<NegociateGymController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void NegociateGymWorldObserver::addRobotToTeleport(int robotId)
{
    robotsToTeleport.insert(robotId);
}

void NegociateGymWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}

void NegociateGymWorldObserver::logSeekTime()
{
    std::cout << "Saving seek time" << std::endl;
    ogzstream f_seektime((gLogDirectoryname + "/seektime_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
    f_seektime << "id\tseekCount\n";
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *rob = gWorld->getRobot(i);
        auto *obs = dynamic_cast<NegociateGymAgentObserver *>(rob->getObserver());
        f_seektime << i << "\t" << obs->getSeekTime() << "\n";
    }
    f_seektime.close();
}
