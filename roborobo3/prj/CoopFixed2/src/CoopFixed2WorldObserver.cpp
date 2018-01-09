/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <CoopFixed2/include/CoopFixed2WorldModel.h>
#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include <boost/algorithm/string.hpp>
#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"


CoopFixed2WorldObserver::CoopFixed2WorldObserver(World *__world) : WorldObserver(__world)
{
    m_world = __world;

    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    CoopFixed2SharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");

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

    m_nbIndividuals = 50;
    gMaxIt = -1;
}


CoopFixed2WorldObserver::~CoopFixed2WorldObserver()
{
    delete m_fitnessLogManager;
};

void CoopFixed2WorldObserver::reset()
{
    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<CoopFixed2Controller * >(m_world->getRobot(0)->getController())->getWeights().size();
    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights);
    m_fitnesses.resize(m_nbIndividuals, 0);

    loadGenomesInRobots(m_individuals);
    resetEnvironment();
}


void CoopFixed2WorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == CoopFixed2SharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        m_curEvaluationInGeneration++;
        resetEnvironment();
    }
    if( m_curEvaluationInGeneration == CoopFixed2SharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
    }
}

void CoopFixed2WorldObserver::stepPost()
{
    /* Teleport robots */
    for (auto rid: robotsToTeleport)
    {
        m_world->getRobot(rid)->unregisterRobot();
        m_world->getRobot(rid)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                   gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
        m_world->getRobot(rid)->registerRobot();
    }
    robotsToTeleport.clear();
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();
    if ((m_generationCount+1) % 1000 == 0)
        saveCustomScreenshot("gen_" + std::to_string(m_generationCount));

}


void CoopFixed2WorldObserver::stepEvolution()
{
    if ((m_generationCount+1) % 1000 == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        m_fitnesses[i] = m_world->getRobot(i)->getWorldModel()->_fitnessValue;
    }
    std::vector<std::pair<int, double>> fitnesses = getSortedFitnesses();
    logFitnesses(fitnesses);
    m_individuals = pyevo.getNextGeneration(m_fitnesses);
    if (m_individuals.empty())
    {
        exit(0);
    }
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> CoopFixed2WorldObserver::getSortedFitnesses() const
{
    std::vector<std::pair<int, double>> fitnesses(m_individuals.size());
    for (int i = 0; i < m_individuals.size(); i++)
    {
        fitnesses[i].first = i;
        fitnesses[i].second = m_fitnesses[i];
    }
    std::sort(fitnesses.begin(), fitnesses.end(),
              [](std::pair<int, double>a, std::pair<int, double>b){return a.second < b.second;});
    return fitnesses;
}

void CoopFixed2WorldObserver::logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses)
{
    unsigned long size = sortedFitnesses.size();
    double sum = std::accumulate(sortedFitnesses.begin(), sortedFitnesses.end(), 0.0,
                                 [](double& a, const std::pair<int, double>& b) -> double {return a + b.second;});
    double mean = sum / size;
    std::vector<double> diff(size);
    std::transform(sortedFitnesses.begin(), sortedFitnesses.end(), diff.begin(),
                   [mean](std::pair<int, double> x) { return x.second - mean; });
    double variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / size;
    std::stringstream out;

    out << m_generationCount << "\t";
    out << size << "\t";
    out << sortedFitnesses[0].second << "\t"; // MIN
    out << sortedFitnesses[size/4].second << "\t"; // 1st quartile
    out << sortedFitnesses[size/2].second << "\t"; // 2nd quartile - Median
    out << sortedFitnesses[(3*size)/4].second << "\t"; // 3rd quartile
    out << sortedFitnesses[size-1].second << "\t"; // Max
    out << mean << "\t";
    out << variance << "\n";
    m_fitnessLogManager->write(out.str());
    m_fitnessLogManager->flush();
}

void CoopFixed2WorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }


    for (auto object: gPhysicalObjects)
    {
        object->resetLocation();
        object->registerObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
    }
}

void CoopFixed2WorldObserver::computeOpportunityImpacts()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<CoopFixed2WorldModel*>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        auto *opp = dynamic_cast<CoopFixed2Opportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(index)->getWorldModel());
            totalInvest += wm->_cooperationLevel;
        }
        for (auto index: opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<CoopFixed2Controller *>(m_world->getRobot(index)->getController());
            wm->onOpportunity = true;
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(totalInvest);
            ctl->increaseFitness(payoff(wm->_cooperationLevel, totalInvest));
        }
    }
}

double CoopFixed2WorldObserver::payoff(const double invest, const double totalInvest) const
{
    double res = 0;
    if (!CoopFixed2SharedData::prisonerDilemma)
    {
        const double min = 0.45; //ensure fitness is always positive
        const double a = 3, B = 4, q = 2, n = 2, c = 0.4;
        const double p = B * std::pow(totalInvest, a) / (std::pow(q, a) + std::pow(totalInvest, a));
        const double share = p / n;
        const double g = std::pow(share, a) / (1 + std::pow(share, a));
        res = min + g - c * invest;
    }
    else
    {
        throw std::runtime_error("Not implemented");
    }
    return res;
}


void CoopFixed2WorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CoopFixed2Opportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void CoopFixed2WorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<CoopFixed2Controller*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void CoopFixed2WorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>>& genomes)
{
    assert(genomes.size() == m_world->getNbOfRobots());
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto* ctl = dynamic_cast<CoopFixed2Controller *>(m_world->getRobot(i)->getController());
        assert(genomes[i].size() == ctl->getWeights().size());
        ctl->loadNewGenome(genomes[i]);
    }

}

void CoopFixed2WorldObserver::addRobotToTeleport(int robotId)
{
    robotsToTeleport.emplace(robotId);
}
