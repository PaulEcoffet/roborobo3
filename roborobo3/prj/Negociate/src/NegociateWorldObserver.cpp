/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <Negociate/include/NegociateWorldModel.h>
#include <Negociate/include/NegociateWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "Negociate/include/NegociateWorldObserver.h"
#include "Negociate/include/NegociateController.h"
#include "Negociate/include/NegociateSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
//#include <cv.hpp>

using boost::algorithm::clamp;


NegociateWorldObserver::NegociateWorldObserver(World *__world) :
        WorldObserver(__world), robotsToTeleport(), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    NegociateSharedData::initSharedData();

    /********************/
    /* Coherence checks */
    /********************/
    if (NegociateSharedData::frictionCoef != 0)
    {
        assert(!NegociateSharedData::fixRobotNb);
    }

    // Assert that no object can be covered twice by the proximity teleport, proximityTeleport == 0 means no constraint
    assert(NegociateSharedData::proximityTeleport >= 0 && NegociateSharedData::proximityTeleport <= (gNbOfPhysicalObjects/2) - 1);
    assert(gNbOfPhysicalObjects % NegociateSharedData::nbCluster == 0);


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
    pyevo.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_nbIndividuals = gInitialNumberOfRobots;
    gMaxIt = -1;

    /* build variability coef distribution */

    variabilityCoef.resize(m_nbIndividuals, 0);

    double startcoef = -NegociateSharedData::fakeCoef;
    double endcoef = +NegociateSharedData::fakeCoef;
    double stepcoef = (endcoef - startcoef) / (m_nbIndividuals-1);
    for (int i = 0; i < m_nbIndividuals; i++) {
        variabilityCoef[i] = startcoef + i * stepcoef;
        assert(variabilityCoef[i] <= NegociateSharedData::fakeCoef + 0.1);
        assert(variabilityCoef[i] >= - NegociateSharedData::fakeCoef - 0.1);
    }
}


NegociateWorldObserver::~NegociateWorldObserver()
{
    m_logall.close();
};

void NegociateWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<NegociateController * >(m_world->getRobot(0)->getController())->getWeights().size();
    std::vector<double> minbounds(nbweights, -20);
    std::vector<double> maxbounds(nbweights, 20);
    std::vector<double> std(nbweights, NegociateSharedData::mutRate);
    std::vector<double> minguess(nbweights, -1);
    std::vector<double> maxguess(nbweights, 1);
    std::vector<double> mutprob(nbweights, NegociateSharedData::mutProb);

    if(NegociateSharedData::fixCoop)
    {
        minbounds[0] = 0;
        maxbounds[0] = 1;
        minguess[0] = 0;
        maxguess[0] = ((NegociateSharedData::meanA / 2) / NegociateSharedData::maxCoop); // Below ESS
        maxguess[0] = 1; // Actually, everything is in Nature
        std[0] = NegociateSharedData::mutCoop;
    }
    bool train = false;
    gProperties.checkAndGetPropertyValue("train", &train, false);
    int split = dynamic_cast<NegociateController*>(gWorld->getRobot(0)->getController())->getSplit();
    if(train)
    {
        /* keep diversity in untrained part of the network */
        mutprob[0] = 1;
        minbounds[0] = minguess[0];
        maxbounds[0] = maxguess[0];
        for(unsigned long i = split; i < std.size(); i++)
        {
            mutprob[i] = 1;
            minbounds[i] = minguess[i];
            maxbounds[i] = maxguess[i];
        }
    }
    else
    {
        mutprob[0] = NegociateSharedData::mutProbCoop;
        for(unsigned long i = split; i < std.size(); i++)
        {
            mutprob[i] = NegociateSharedData::mutProbNegociate;
        }
    }
    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights, minbounds, maxbounds, minguess, maxguess, std, mutprob);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();
}


void NegociateWorldObserver::stepPre()
{
    if (m_curEvaluationIteration == NegociateSharedData::evaluationTime || endEvaluationNow)
    {
        m_curEvaluationIteration = 0;
        endEvaluationNow = false;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<NegociateWorldModel*>(m_world->getRobot(i)->getWorldModel());
            m_fitnesses[i] += wm->_fitnessValue;
            m_curfitnesses[i] = wm->_fitnessValue;
        }
        logFitnesses(m_curfitnesses);
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;

        resetEnvironment();
    }
    if (m_curEvaluationInGeneration == NegociateSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
    }

    for (int i = 0; i < gInitialNumberOfRobots; i++)
    {
        auto* rob = m_world->getRobot(i);
        auto* wm = rob->getWorldModel();
        if (not wm->isAlive())
        {
            wm->_desiredTranslationalValue = 0;
            wm->_desiredRotationalVelocity = 0;
            if (NegociateSharedData::tau != 0 && random() < 1.0 / NegociateSharedData::tau)
            {
                wm->setAlive(true);
                rob->findRandomLocation(gAgentsInitAreaX,
                                        gAgentsInitAreaX + gAgentsInitAreaWidth,
                                        gAgentsInitAreaY,
                                        gAgentsInitAreaY + gAgentsInitAreaHeight);
            }
        }
    }
}

void NegociateWorldObserver::stepPost()
{
    /* Plays */
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();

    /* Move */
    /* Teleport robots */
    if (NegociateSharedData::teleportRobots)
    {
        for (auto rid: robotsToTeleport)
        {
            m_world->getRobot(rid)->unregisterRobot();
            m_world->getRobot(rid)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                       gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
            m_world->getRobot(rid)->registerRobot();
        }
    }


    for (auto id: objectsToTeleport)
    {
        double prevx = gPhysicalObjects[id]->getXReal();
        double prevy = gPhysicalObjects[id]->getYReal();
        gPhysicalObjects[id]->unregisterObject();
        gPhysicalObjects[id]->setCoordinates(emptyX, emptyY);
        emptyX = prevx;
        emptyY = prevy;
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();
    robotsToTeleport.clear();

    if ((m_generationCount + 1) % NegociateSharedData::logEveryXGen == 0)
    {
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
        {
            std::cout << "**********************************\n";
            std::cout << "TIME TO LOG\n";
            std::cout << "**********************************\n";
            if (NegociateSharedData::takeVideo)
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
            m_logall << "eval\titer\tid1\tfakeCoef1\ttrueCoop1\tid2\tfakeCoef2\ttrueCoop2\tAccept1\tAccept2\n";
        }
        if(NegociateSharedData::takeVideo)
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
    else if ((m_generationCount + 1) % NegociateSharedData::logEveryXGen == 1 && m_curEvaluationIteration == 0)
    {
        m_logall.close();
        //outvid.release();
    }

    m_curEvaluationIteration++;
}


void NegociateWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % NegociateSharedData::logEveryXGen == 0)
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

std::vector<std::pair<int, double>> NegociateWorldObserver::getSortedFitnesses() const
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

void NegociateWorldObserver::logFitnesses(const std::vector<double> &curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(i)->getWorldModel());
        out << m_generationCount << "\t"
            << i << "\t"
            << m_curEvaluationInGeneration << "\t"
            << cur_wm->fakeCoef << "\t"
            << curfitness[i] << "\n";
    }
    m_fitnessLogManager << out.str();
    m_fitnessLogManager.flush();
}

void NegociateWorldObserver::resetEnvironment()
{
    endEvaluationNow = false;
    emptyX = 90;
    emptyY = 90;
    nbOfRobotsWhoPlayed = 0;
    for (auto object: gPhysicalObjects)
    {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
        auto *rwm = dynamic_cast<NegociateWorldModel *>(robot->getWorldModel());
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
        if (NegociateSharedData::tpToNewObj)
        {
            robot->getWorldModel()->_agentAbsoluteOrientation = 0;
        }
        auto *wm = dynamic_cast<NegociateWorldModel *>(robot->getWorldModel());
        if (NegociateSharedData::fakeRobots)
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

void NegociateWorldObserver::computeOpportunityImpacts()
{
    const double b = NegociateSharedData::b;
    // Mark all robots as not on an cooperation opportunity
    mark_all_robots_as_alone();

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<NegociateOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (n != 0)
        {
            if (NegociateSharedData::fixRobotNb && n > 2)
            {
                n = 2;
            }

            mark_robots_on_opp(opp);

            // If we only give reward for the first two robots
            if (NegociateSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
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
                auto *const wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                agentsCoop[i] = wm->getCoop(true);
                agentsFake[i] = wm->fakeCoef;
                totalInvest += coop;
                totalA += wm->selfA;
                i++;
            }



            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                wm->appendOwnInvest(coop);

                if (NegociateSharedData::onlyOtherInTotalInv)
                {
                    wm->appendTotalInvest(totalInvest - coop);
                }
                else
                {
                    wm->appendTotalInvest(totalInvest);
                }
            }

            i = 0;
            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *const ctl = dynamic_cast<NegociateController *>(m_world->getRobot(*index)->getController());
                bool accept = ctl->acceptPlay();
                agentsAccept[i] = accept;
                everyone_agree = everyone_agree && accept;
                i++;
            }
            if((m_generationCount + 1) % NegociateSharedData::logEveryXGen == 0
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

                    auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                    double coop = wm->getCoop();
                    if (everyone_agree)
                    {
                        double curpayoff = payoff(coop, totalInvest, n, wm->selfA, b) /
                                           NegociateSharedData::nbEvaluationsPerGeneration;

                        if (NegociateSharedData::doNotKill)
                        {
                            if (wm->_fitnessValue == 0 &&
                                curpayoff != 0) // curpayoff is VERY unlikely to be exactly 0 but prevent bugs
                            {
                                wm->_fitnessValue = curpayoff;
                                nbOfRobotsWhoPlayed++;
                                if (nbOfRobotsWhoPlayed == m_nbIndividuals and NegociateSharedData::tau == 0)
                                {
                                    std::cout << "evaluation shorten, everyone has a payoff" << std::endl;
                                    endEvaluationNow = true;
                                }
                            }
                        }
                        else
                        {
                            wm->_fitnessValue += curpayoff;
                            wm->setAlive(false);
                            nbOfRobotsWhoPlayed++;
                            if (nbOfRobotsWhoPlayed == m_nbIndividuals and NegociateSharedData::tau == 0)
                            {
                                std::cout << "evaluation shorten, everyone has a payoff" << std::endl;
                                endEvaluationNow = true;
                            }

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

void NegociateWorldObserver::mark_robots_on_opp(NegociateOpportunity *opp) const
{
    int arrival = 1;
    for (auto index : opp->getNearbyRobotIndexes())
    {
        auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(index)->getWorldModel());
        wm->onOpportunity = true;
        wm->opp = opp;
        wm->nbOnOpp = opp->getNbNearbyRobots();
        wm->arrival = arrival;
        arrival++;
    }
}

void NegociateWorldObserver::mark_all_robots_as_alone() const
{
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->opp = nullptr;
        wm->nbOnOpp = 0;
        wm->arrival = 0;
        wm->punishment = 0;
    }
}

void NegociateWorldObserver::reward_lonely(double sum_payoff, int nb_payoffs) const
{
        for (int i = 0; i < m_world->getNbOfRobots(); i++)
        {
            auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(i)->getWorldModel());
            if (!wm->isPlaying())
            {
                wm->_fitnessValue += std::min(NegociateSharedData::sigma, 0.8 * sum_payoff / nb_payoffs);
            }
        }
}

static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound)/(1 + exp(-slope*(x - inflexionPoint)));
}


double NegociateWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                       const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);
    if (NegociateSharedData::atLeastTwo and n < 2)
    {
        res = 0; // No payoff if you are alone
    }
    else
    {
        res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;
    }
    if (NegociateSharedData::frictionCoef > 0)
    {
        res *= (1-sigmoid(n, 0, 1, NegociateSharedData::frictionCoef, NegociateSharedData::frictionInflexionPoint));
    }
    if (NegociateSharedData::temperature > 0)
    {
        res = std::exp(res / NegociateSharedData::temperature);
    }
    return res;
}



void NegociateWorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<NegociateOpportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void NegociateWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<NegociateController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void NegociateWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<NegociateController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void NegociateWorldObserver::addRobotToTeleport(int robotId)
{
    robotsToTeleport.insert(robotId);
}

void NegociateWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}
