/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <Skilled/include/SkilledWorldModel.h>
#include <Skilled/include/SkilledWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "Skilled/include/SkilledWorldObserver.h"
#include "Skilled/include/SkilledController.h"
#include "Skilled/include/SkilledSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
#include <Skilled/include/SkilledTournamentWorldObserver.h>

using boost::algorithm::clamp;


SkilledTournamentWorldObserver::SkilledTournamentWorldObserver(World *__world) :
        WorldObserver(__world), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    SkilledSharedData::initSharedData();


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
    boost::math::normal normal(1, SkilledSharedData::fakeCoefStd);

    double minquantile = boost::math::cdf(normal, 1 - SkilledSharedData::fakeCoef);
    double maxquantile = boost::math::cdf(normal, 1 + SkilledSharedData::fakeCoef);
    double stepquantile = (maxquantile - minquantile) / (m_nbIndividuals - 1);
    double curquantile = minquantile;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        variabilityCoef[i] = boost::math::quantile(normal, curquantile);
        assert(variabilityCoef[i] <= 1 + SkilledSharedData::fakeCoef + 0.1);
        assert(variabilityCoef[i] >= 1 - SkilledSharedData::fakeCoef - 0.1);

        curquantile += stepquantile;
    }

}


SkilledTournamentWorldObserver::~SkilledTournamentWorldObserver()
{
    delete m_fitnessLogManager;
}

void SkilledTournamentWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<SkilledController * >(m_world->getRobot(0)->getController())->getWeights().size();
    std::vector<double> minbounds(nbweights, -10);
    std::vector<double> maxbounds(nbweights, 10);
    std::vector<double> minguess(nbweights, -1);
    std::vector<double> maxguess(nbweights, 1);

    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights, minbounds, maxbounds, minguess, maxguess);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();

    std::uniform_int_distribution<int> randomNbOpp(0, gInitialNumberOfRobots - 1);
    std::uniform_real_distribution<double> randomInvPerAg(0, SkilledSharedData::maxCoop);
    std::normal_distribution<double> randomSmallVar(0, 1);
    std::normal_distribution<double> randomSmallN(0, 3);
    do
    {
        if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0)
        {
            std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
            std::ofstream genfile(path);
            genfile << json(m_individuals);
        }

        m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
        loadGenomesInRobots(m_individuals);
        for (int i = 0; i < gInitialNumberOfRobots; i++)
        {
            auto ctl = dynamic_cast<SkilledController *>(gWorld->getRobot(i)->getController());
            for (int iter = 0; iter < SkilledSharedData::evaluationTime; iter++)
            {

                int nbOnOpp1 = randomNbOpp(engine);
                double invPerAg1 = randomInvPerAg(engine);
                bool cost1 = false;
                double ownInv1 = ctl->getCoop(nbOnOpp1);
                double score1 = ctl->computeScore(cost1, nbOnOpp1, ownInv1, invPerAg1 * nbOnOpp1);
                double payoff1 = SkilledWorldObserver::payoff(ownInv1, invPerAg1 * nbOnOpp1 + ownInv1, nbOnOpp1 + 1,
                                                              SkilledSharedData::meanA, SkilledSharedData::b) -
                                 cost1 * SkilledSharedData::cost;

                int nbOnOpp2 = clamp(nbOnOpp1 + std::round(randomSmallN(engine)), 0, gInitialNumberOfRobots - 1);
                double invPerAg2 = clamp(invPerAg1 + randomSmallVar(engine), 0, SkilledSharedData::maxCoop);
                bool cost2 = false;
                double ownInv2 = ctl->getCoop(nbOnOpp2);
                double score2 = ctl->computeScore(cost2, nbOnOpp2, ownInv2, invPerAg2 * nbOnOpp2);
                double payoff2 = SkilledWorldObserver::payoff(ownInv2, invPerAg2 * nbOnOpp2 + ownInv2, nbOnOpp2 + 1,
                                                              SkilledSharedData::meanA, SkilledSharedData::b) -
                                 cost2 * SkilledSharedData::cost;
                /*//
                std::cout << "rob: " << i << ", iter: " << iter << "\n";
                std::cout << "nbOnOpp1: " << nbOnOpp1 << ", invPerAg1: " << invPerAg1
                          << ", cost1:" << cost1 << ", ownInv1: " << ownInv1 << "\n";
                std::cout << "score1: " << score1 << ", payoff1: " << payoff1 << std::endl;

                std::cout << "nbOnOpp2: " << nbOnOpp2 << ", invPerAg2: " << invPerAg2
                          << ", cost2:" << cost2 << ", ownInv2: " << ownInv2 << "\n";
                std::cout << "score2: " << score1 << ", payoff2: " << payoff2 << std::endl;
                 //*/
                if (score1 > score2)
                {
                    m_fitnesses[i] += payoff1 >= payoff2;
                }
                else
                {
                    m_fitnesses[i] += payoff2 >= payoff1;
                }
            }
        }
        logFitnesses(m_fitnesses);
        m_individuals = pyevo.getNextGeneration(m_individuals, m_fitnesses);
        m_generationCount++;
    } while (!m_individuals.empty());
    exit(0);
}


void SkilledTournamentWorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == SkilledSharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<SkilledWorldModel *>(m_world->getRobot(i)->getWorldModel());
            m_fitnesses[i] += wm->_fitnessValue;
            m_curfitnesses[i] = wm->_fitnessValue;
        }
        logFitnesses(m_curfitnesses);
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;

        resetEnvironment();
    }
    if (m_curEvaluationInGeneration == SkilledSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
    }

    /* Shall we log? */
    if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0)
    {
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
        {
            if (m_logall.is_open())
            {
                m_logall.close();
            }
            m_logall.open(gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt");
            m_logall
                    << "eval\titer\tid\ta\tfakeCoef\tplaying\toppId\tnbOnOpp\tcurCoopNoCoef\totherCoop\n";
        }
    }
    else if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 1 && m_logall.is_open())
    {
        m_logall.close();
    }
}


void SkilledTournamentWorldObserver::logAgent(SkilledWorldModel *wm)
{
    if (!m_logall.is_open())
    {
        return;
    }

    int nbOnOpp = wm->opp->countCurrentRobots();
    m_logall << m_curEvaluationInGeneration << "\t"
             << m_curEvaluationIteration << "\t"
             << wm->getId() << "\t"
             << wm->selfA << "\t"
             << wm->fakeCoef << "\t"
             << wm->isPlaying() << "\t"
             << ((wm->opp != nullptr) ? wm->opp->getId() : -1) << "\t"
             << nbOnOpp << "\t"
             << wm->getCoop(nbOnOpp - 1, true) << "\t"
             << wm->opp->getCurInv() - wm->getCoop(nbOnOpp - 1)
             << "\n";
}


void SkilledTournamentWorldObserver::stepPost()
{
    /* Plays */

    for (auto id: objectsToTeleport)
    {
        gPhysicalObjects[id]->unregisterObject();
        gPhysicalObjects[id]->resetLocation();
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();
    if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0)
    {
        if (SkilledSharedData::takeVideo and m_curEvaluationInGeneration == 0)
        {
            saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
        }
    }
}

void SkilledTournamentWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0)
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

std::vector<std::pair<int, double>> SkilledTournamentWorldObserver::getSortedFitnesses() const
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

void SkilledTournamentWorldObserver::logFitnesses(const std::vector<double> &curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<SkilledWorldModel *>(m_world->getRobot(i)->getWorldModel());
        out << m_generationCount << "\t"
            << i << "\t"
            << m_curEvaluationInGeneration << "\t"
            << cur_wm->fakeCoef << "\t"
            << curfitness[i] << "\n";
    }
    m_fitnessLogManager->write(out.str());
    m_fitnessLogManager->flush();
}

void SkilledTournamentWorldObserver::resetEnvironment()
{
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
        object->registerObject();
        auto *lionobj = dynamic_cast<SkilledOpportunity *>(object);
        lionobj->reset();
    }

    // Randomize the fakeList so that the last fakeRobots aren't necessarily the ones who cooperate the most.
    std::shuffle(variabilityCoef.begin(), variabilityCoef.end(), engine);

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->getWorldModel()->_agentAbsoluteOrientation = 0;

        auto *wm = dynamic_cast<SkilledWorldModel *>(robot->getWorldModel());
        wm->reset();
        if (SkilledSharedData::fakeRobots)
        {
            wm->fakeCoef = variabilityCoef[iRobot];
            assert(wm->fakeCoef <= 1 + SkilledSharedData::fakeCoef + 0.1);
            assert(wm->fakeCoef >= 1 - SkilledSharedData::fakeCoef - 0.1);
        }
        else
        {
            wm->fakeCoef = 1.0;
        }
        wm->setNewSelfA();
    }
}


static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound) / (1 + exp(-slope * (x - inflexionPoint)));
}


double
SkilledTournamentWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                       const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);

    res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;

    if (gVerbose)
    {
        std::cout << "x:" << invest << ", x0:" << x0 << ", n:" << n << ", res:" << res << "\n";
    }
    if (SkilledSharedData::frictionCoef > 0)
    {
        res *= (1 - sigmoid(n, 0, 1, SkilledSharedData::frictionCoef, SkilledSharedData::frictionInflexionPoint));
    }

    return res;
}


void SkilledTournamentWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<SkilledController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void SkilledTournamentWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<SkilledController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void SkilledTournamentWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}
