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
#include "Agents/Robot.h"

using boost::algorithm::clamp;


SkilledWorldObserver::SkilledWorldObserver(World *__world) :
        WorldObserver(__world), objectsToTeleport(), variabilityCoef()
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_trueCurEvaluationInGeneration = 0;
    m_generationCount = 0;

    SkilledSharedData::initSharedData();


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

    variabilityCoef.resize(m_nbIndividuals, 1);


    if (SkilledSharedData::normalCoef)
    {
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
    else
    {
        double min = 1 - SkilledSharedData::fakeCoef;
        double max = 1 + SkilledSharedData::fakeCoef;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            variabilityCoef[i] = min + (max - min) * (double) i / (m_nbIndividuals - 1);
        }
    }
}


SkilledWorldObserver::~SkilledWorldObserver()
{
    cleanup();
}

void SkilledWorldObserver::cleanup()
{
    std::cout << "Bien fermé pour le WorldObserver" << std::endl;
    m_logall.close();
    scorelogger.close();
}

void SkilledWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;

    // Init fitness
    clearRobotFitnesses();
    m_curnbparticipation.resize(m_nbIndividuals, 0);


    // create individuals
    int nbweights = dynamic_cast<SkilledController * >(m_world->getRobot(0)->getController())->getWeights().size();
    std::vector<double> minbounds(nbweights, -10);
    std::vector<double> maxbounds(nbweights, 10);
    std::vector<double> minguess(nbweights, -1);
    std::vector<double> maxguess(nbweights, 1);
    std::vector<double> std(nbweights, SkilledSharedData::normalMut);
    std::vector<double> mutprob(nbweights, SkilledSharedData::mutProb);
    for (auto i = minbounds.size() - 1 - 3; i < minbounds.size(); i++)
    {
        minbounds[i] = 0;
        maxbounds[i] = 1;
        minguess[i] = 0;
        maxguess[i] = 0.1;
    }

    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights, minbounds, maxbounds, minguess, maxguess, std, mutprob);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();

    if (isLoggingTime())
    {
        std::cout << "coucou" << std::endl;
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
        {
            std::cout << "lolilol" << std::endl;
            m_logall.close();
            m_logall.open((gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
            m_logall
                    << "eval\titer\tid\ta\tfakeCoef\tplaying\toppId\tnbOnOpp\tcurCoopNoCoef\totherCoop\n";
            scorelogger.openNewLog(m_generationCount);
            scorelogger.updateEval(m_curEvaluationInGeneration);
        }
    }
}


void SkilledWorldObserver::stepPre()
{
    m_curEvaluationIteration++;
    bool mustresetEnv = false;
    /* NEW EVALUATION */
    if (m_curEvaluationIteration == SkilledSharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = dynamic_cast<SkilledWorldModel *>(m_world->getRobot(i)->getWorldModel());
            /* Only add fitness if the robot is in his participation run */
            if (m_curnbparticipation[i] <= SkilledSharedData::nbEvaluationsPerGeneration)
            {
                m_fitnesses[i] += wm->_fitnessValue;
                m_curfitnesses[i] = wm->_fitnessValue;
            }
        }
        clearRobotFitnesses();
        m_curEvaluationInGeneration = *std::min_element(std::begin(m_curnbparticipation),
                                                        std::end(m_curnbparticipation));
        std::cout << "Cur Ev:" << m_curEvaluationInGeneration << std::endl;
        mustresetEnv = true;
        m_trueCurEvaluationInGeneration++;
        scorelogger.updateEval(m_trueCurEvaluationInGeneration);

    }

    /* NEW GENERATION */
    if (m_curEvaluationInGeneration == SkilledSharedData::nbEvaluationsPerGeneration)
    {
        m_logall.close();  // Cur log must necessarily be closed.
        scorelogger.close();
        logFitnesses(m_fitnesses);
        printCoopStats();
        stepEvolution();
        m_curEvaluationInGeneration = 0;
        m_trueCurEvaluationInGeneration = 0;
        m_curnbparticipation = std::vector<int>(m_nbIndividuals, 0);
        m_generationCount++;
        mustresetEnv = true;
        /* Shall we log? */
        if (isLoggingTime())
        {
            if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0)
            {
                m_logall.close();
                m_logall.open((gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt.gz").c_str());
                m_logall
                        << "eval\titer\tid\ta\tfakeCoef\tplaying\toppId\tnbOnOpp\tcurCoopNoCoef\totherCoop\n";
                scorelogger.openNewLog(m_generationCount);
                scorelogger.updateEval(m_trueCurEvaluationInGeneration);
            }
        }
        else if (isJustAfterLoggingTime())
        {
            m_logall.close();
            scorelogger.close();
        }

    }
    if (mustresetEnv)
        resetEnvironment();

    scorelogger.updateIter(m_curEvaluationIteration);
}

bool SkilledWorldObserver::isLoggingTime() const
{
    return (((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0) && (m_curEvaluationInGeneration < 5));
}

bool SkilledWorldObserver::isJustAfterLoggingTime() const
{
    return ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 1);
}

void SkilledWorldObserver::logAgent(SkilledWorldModel *wm)
{
    if (isLoggingTime())
    {
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
}


void SkilledWorldObserver::stepPost()
{
    /* Plays */
    if (!SkilledSharedData::asyncPlay)
    {
        for (int i = 0; i < gNbOfRobots; i++)
        {
            dynamic_cast<SkilledController *>(gWorld->getRobot(i)->getController())->play_and_fitness();
        }
    }

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

void SkilledWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % SkilledSharedData::logEveryXGen == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    std::vector<double> normfitness(m_nbIndividuals);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        normfitness[i] = m_fitnesses[i] / m_curnbparticipation[i];
    }
    m_individuals = pyevo.getNextGeneration(m_individuals, normfitness);
    if (m_individuals.empty())
    {
        cleanup();
        exit(0);
    }
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    m_curnbparticipation = std::vector<int>(m_nbIndividuals, 0);

    loadGenomesInRobots(m_individuals);
    clearRobotFitnesses();


}

std::vector<std::pair<int, double>> SkilledWorldObserver::getSortedFitnesses() const
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

void SkilledWorldObserver::logFitnesses(const std::vector<double> &curfitness)
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
    m_fitnessLogManager << out.str();
    m_fitnessLogManager.flush();
}

void SkilledWorldObserver::resetEnvironment()
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
        if (auto *skillobj = dynamic_cast<SkilledOpportunity *>(object))
            skillobj->reset();
        else
            exit(-1);
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
        wm->setAlive(false);
    }

    this->setWhichRobotsPlay();
}


static double sigmoid(double x, double lowerbound, double upperbound, double slope, double inflexionPoint)
{
    return lowerbound + (upperbound - lowerbound) / (1 + exp(-slope * (x - inflexionPoint)));
}


static double bellcurve(double x, double mu, double sigma)
{

    return std::exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}


double SkilledWorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a,
                                    const double b)
{
    double res = 0;
    const double x0tot = (totalInvest - invest);

    // apply benefits
    res = (a * totalInvest + b * x0tot) / n;
    if (SkilledSharedData::maxTwo && n != 2)
    {
        res = 0;
    }

    // apply friction on benefits
    if (SkilledSharedData::nControl == 1)
    {
        res *= (1 - sigmoid(n, 0, 1, SkilledSharedData::frictionCoef, SkilledSharedData::frictionInflexionPoint));
    }
    else if (SkilledSharedData::nControl == 2)
    {
        // Divide by bellcurve
        res *= bellcurve(n, SkilledSharedData::nOpti, SkilledSharedData::nTolerance);
    }
    else if (SkilledSharedData::nControl == 3)
    {
        // Divide by bellcurve for the whole payoff, not only the benefits
        res -= 0.5 * invest * invest;
        res *= bellcurve(n, SkilledSharedData::nOpti, SkilledSharedData::nTolerance);
    }
    else if (SkilledSharedData::nControl == 4)
    {
        /* True prisoner's dilemma with friction */
        if (n >= 2)
        {
            res = b * x0tot / (n - 1);
            res -= 0.5 * invest * invest;
        }
        else
        {
            res = 0;
        }
        res *= bellcurve(n, SkilledSharedData::nOpti, SkilledSharedData::nTolerance);
    }
    else if (SkilledSharedData::nControl == 5)
    {
        /* True Prisoner's dilemma without friction */
        if (n >= 2)
        {
            res = b * x0tot / (n - 1);
            res -= 0.5 * invest * invest;
        }
        else
        {
            res = 0;
        }
    }
    else if (SkilledSharedData::nControl == 6)
    {
        /* by product benefits */
        res = a * invest + b * x0tot / std::max(n - 1, 1);
        res -= 0.5 * invest * invest;
        res *= bellcurve(n, SkilledSharedData::nOpti, SkilledSharedData::nTolerance);
    }

    // Apply cost
    if (SkilledSharedData::nControl < 3)
    {
        res -= 0.5 * invest * invest;
    }
    assert(res == res); // Test if not nan
    return res;
}


void SkilledWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<SkilledController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void SkilledWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{

    if (genomes.size() != m_nbIndividuals)
    {
        std::cout << "genomes: " << genomes.size() << ", m_nbIndividuals : " << m_nbIndividuals << "\n";
        exit(1);
    }
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<SkilledController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void SkilledWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.insert(id);
}

void SkilledWorldObserver::setWhichRobotsPlay()
{
    std::vector<int> individuals_indexes(m_nbIndividuals);
    std::iota(std::begin(individuals_indexes), std::end(individuals_indexes), 0); // Fill with 0, 1, ..., 99.
    std::vector<int> individual_scores = individuals_indexes;
    std::shuffle(individual_scores.begin(), individual_scores.end(), engine);
    /* Put the ones that have participated less first, then the ones who participated the most
     * If they participated the same amount, pick them randomly (score which is random) */
    std::sort(individuals_indexes.begin(), individuals_indexes.end(),
              [individual_scores, this](int a, int b)
              {
                  if (m_curnbparticipation[a] == m_curnbparticipation[b])
                  {
                      return individual_scores[a] < individual_scores[b];
                  }
                  return m_curnbparticipation[a] < m_curnbparticipation[b];
              });
    for (size_t i = 0; i < gNbOfRobots; i++)
    {
        auto wm = dynamic_cast<SkilledWorldModel *>(gWorld->getRobot(i)->getWorldModel());
        wm->setAlive(false);
        auto rob = gWorld->getRobot(i);
        rob->unregisterRobot();
        rob->setCoord(1, 1);
        rob->setCoordReal(1, 1);
        rob->registerRobot();

    }
    for (auto index = individuals_indexes.begin();
         index != (individuals_indexes.begin() + SkilledSharedData::maxPlayer); index++)
    {
        auto wm = dynamic_cast<SkilledWorldModel *>(gWorld->getRobot(*index)->getWorldModel());
        wm->setAlive(true);
        m_world->getRobot(*index)->unregisterRobot();
        m_world->getRobot(*index)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                      gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
        m_world->getRobot(*index)->registerRobot();
        m_curnbparticipation[*index]++;

    }
}

SkilledScoreLogger *SkilledWorldObserver::getScoreLogger()
{
    return &scorelogger;
}

bool SkilledWorldObserver::logScore()
{
    return isLoggingTime() && SkilledSharedData::logScore;
}

void SkilledWorldObserver::printCoopStats()
{
    std::vector<double> coops(m_nbIndividuals, 0);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto *cur_wm = dynamic_cast<SkilledWorldModel *>(m_world->getRobot(i)->getWorldModel());
        coops[i] = cur_wm->getCoop(2, true);
    }
    std::sort(coops.begin(), coops.end());
    std::cout << "coop quartiles: " << coops[0] << ", " << coops[m_nbIndividuals / 4] << ", "
              << coops[m_nbIndividuals / 2] << ", " << coops[3 * m_nbIndividuals / 4] << ", "
              << coops[m_nbIndividuals - 1] << std::endl;
}
