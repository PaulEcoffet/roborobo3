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
        WorldObserver(__world), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    LionSharedData::initSharedData();


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

    variabilityCoef.resize(m_nbIndividuals, 1);


    if (LionSharedData::normalCoef)
    {
        boost::math::normal normal(1, LionSharedData::fakeCoefStd);
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
    else
    {
        double min = 1 - LionSharedData::fakeCoef;
        double max = 1 + LionSharedData::fakeCoef;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            variabilityCoef[i] = min + (max - min) * (double)i / (m_nbIndividuals - 1);
        }
    }
}


LionWorldObserver::~LionWorldObserver()
{
    delete m_fitnessLogManager;
}

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

    /* Shall we log? */
    if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 0)
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
    else if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 1 && m_logall.is_open())
    {
        m_logall.close();
    }
}


void LionWorldObserver::logAgent(LionWorldModel *wm)
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


void LionWorldObserver::stepPost()
{
    /* Plays */
    if(!LionSharedData::asyncPlay)
    {
        for (int i = 0; i < gNbOfRobots; i++)
        {
            dynamic_cast<LionController *>(gWorld->getRobot(i)->getController())->play_and_fitness();
        }
    }

    for (auto id: objectsToTeleport)
    {
        gPhysicalObjects[id]->unregisterObject();
        gPhysicalObjects[id]->resetLocation();
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();
    if ((m_generationCount + 1) % LionSharedData::logEveryXGen == 0)
    {
        if (LionSharedData::takeVideo and m_curEvaluationInGeneration == 0)
        {
            saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
        }
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
    }


    for (auto *object: gPhysicalObjects)
    {
        object->resetLocation();
        object->registerObject();
        auto* lionobj = dynamic_cast<LionOpportunity*>(object);
        lionobj->reset();
    }

    // Randomize the fakeList so that the last fakeRobots aren't necessarily the ones who cooperate the most.
    std::shuffle(variabilityCoef.begin(), variabilityCoef.end(), engine);

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->getWorldModel()->_agentAbsoluteOrientation = 0;

        auto *wm = dynamic_cast<LionWorldModel *>(robot->getWorldModel());
        wm->reset();
        if (LionSharedData::fakeRobots)
        {
            wm->fakeCoef = variabilityCoef[iRobot];
            assert(wm->fakeCoef <= 1 + LionSharedData::fakeCoef + 0.1);
            assert(wm->fakeCoef >= 1 - LionSharedData::fakeCoef - 0.1);
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


static const double invsqrt2pi = 1.0 / std::sqrt(2 * M_PI);

static double bellcurve(double x, double mu, double sigma)
{

    return invsqrt2pi * 1.0 / sigma * std::exp(- ((x - mu) * (x - mu)) / (2 * sigma * sigma));
}


double LionWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
        const double b)
{
    double res = 0;
    const double x0 = (totalInvest - invest);

    res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest;
    if (LionSharedData::maxTwo && n != 2)
    {
        res = 0;
    }

    if (gVerbose)
    {
        //std::cout << "x:" << invest << ", x0:" << x0 << ", n:" << n << ", res:" << res << "\n";
    }
    if (LionSharedData::nControl == 1)
    {
        res *= (1 - sigmoid(n, 0, 1, LionSharedData::frictionCoef, LionSharedData::frictionInflexionPoint));
    }
    else if (LionSharedData::nControl == 2)
    {
        res *= bellcurve(n, LionSharedData::nOpti, LionSharedData::nTolerance);
    }

    return res;
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

void LionWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}
