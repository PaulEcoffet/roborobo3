//
// Created by paul on 30/10/17.
//

#include <PartnerControl/include/PartnerControlWorldModel.h>
#include <Utilities/Graphics.h>
#include <PartnerControl/include/PartnerControlAnalysisWorldObserver.h>
#include <PartnerControl/include/PartnerControlWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "RoboroboMain/main.h"
#include "PartnerControl/include/PartnerControlWorldObserver.h"
#include "PartnerControl/include/PartnerControlController.h"
#include "PartnerControl/include/PartnerControlSharedData.h"


PartnerControlWorldObserver::PartnerControlWorldObserver(World *__world) : WorldObserver(__world)
{
    m_world = __world;

    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;
    m_curInd = 0;

    PartnerControlSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");

    m_observer = new LogManager(gLogDirectoryname + "/observer.txt");
    m_observer->write("gen\tind\teval\titer\tonOpp\tcoop\n");

    std::vector<std::string> url;
    if (gRemote == "")
    {
        std::cerr << "[WARNING] gRemote needs to be defined.";
        url.emplace_back("127.0.0.1");
        url.emplace_back("1703");
    }
    boost::split(url, gRemote, boost::is_any_of(":"));
    pycma.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_nbIndividuals = 50;
    gMaxIt = -1;
}


PartnerControlWorldObserver::~PartnerControlWorldObserver()
{
    delete m_fitnessLogManager;
    delete m_observer;
};

void PartnerControlWorldObserver::reset()
{
    // Init environment
    initOpportunities();

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<PartnerControlController * >(m_world->getRobot(0)->getController())->getWeights().size();
    m_individuals = pycma.initCMA(m_nbIndividuals, nbweights);
    m_fitnesses.resize(m_nbIndividuals, 0);

    activateOnlyRobot(m_curInd);
    resetEnvironment();
}

void PartnerControlWorldObserver::initOpportunities()
{
    double curCoop =  m_curEvaluationInGeneration * PartnerControlSharedData::maxCoop / ((double) PartnerControlSharedData::nbEvaluationsPerGeneration - 1);
    auto &physObjList = gPhysicalObjects;
    for (auto *physicalObject : physObjList)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(physicalObject);
        opp->setCoopValue(curCoop);

    }
}

void PartnerControlWorldObserver::stepPre()
{
    monitorPopulation();
    computeOpportunityImpact();
    clearOpportunityNearbyRobots();

    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == PartnerControlSharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        m_curEvaluationInGeneration++;
        resetEnvironment();
    }
    if( m_curEvaluationInGeneration == PartnerControlSharedData::nbEvaluationsPerGeneration)
    {
        m_fitnesses[m_curInd] = m_world->getRobot(0)->getWorldModel()->_fitnessValue;
        m_world->getRobot(0)->getWorldModel()->_fitnessValue = 0;
        m_curInd++;
        m_curEvaluationInGeneration = 0;
        if (m_curInd < m_individuals.size())
        {
            activateOnlyRobot(m_curInd);
        }
    }
    if (m_curInd == m_individuals.size())
    {
        stepEvolution();
        m_curInd = 0;
        activateOnlyRobot(m_curInd);
        m_generationCount++;
    }
}

void PartnerControlWorldObserver::monitorPopulation() const
{
    if ((m_generationCount + 1) % PartnerControlSharedData::takeVideoEveryGeneration == 0 && m_curInd % 25 == 0)
    {
        std::string name = "gen_" + std::to_string(m_generationCount) + "_ind_" + std::to_string(m_curInd);
        saveCustomScreenshot(name);
    }

    if ((m_generationCount + 1) % PartnerControlSharedData::takeVideoEveryGeneration == 0)
    {
        std::stringstream out;
        auto *wm = dynamic_cast<PartnerControlWorldModel *>(m_world->getRobot(0)->getWorldModel());
        out << m_generationCount << "\t";
        out << m_curInd << "\t";
        out << m_curEvaluationInGeneration << "\t";
        out << m_curEvaluationIteration << "\t";
        out << wm->onOpportunity << "\t";
        if (wm->onOpportunity)
        {
            out << wm->lastTotalInvest.back() << "\n";
        }
        else
        {
            out << "0\n";
        }
        m_observer->write(out.str());

    }
}

void PartnerControlWorldObserver::stepEvolution()
{
    std::vector<std::pair<int, double>> fitnesses = getSortedFitnesses();
    logFitnesses(fitnesses);
    m_individuals = pycma.getNextGeneration(m_fitnesses);
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> PartnerControlWorldObserver::getSortedFitnesses() const
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

void PartnerControlWorldObserver::logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses)
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

void PartnerControlWorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }

    initOpportunities();

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

void PartnerControlWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<PartnerControlWorldModel*>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<PartnerControlWorldModel*>(m_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<PartnerControlController*>(m_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            ctl->increaseFitness(payoff(wm->_cooperationLevel, opp->getCoop() + wm->_cooperationLevel));
        }
    }
}

double PartnerControlWorldObserver::payoff(const double invest, const double totalInvest) const
{
    double res;
    if (!PartnerControlSharedData::gaussianPayoff)
    {
        /*
        const int nbRobots = 2;
        const double coeff = PartnerControlSharedData::constantK / (1.0 + pow(nbRobots - 2, 2)); // \frac{k}{1+(n-2)^2}
        res = coeff * pow(totalInvest, PartnerControlSharedData::constantA) - invest;
         */
        const double a = 3, B = 2, q = 1, n = B, c = 0.2;
        const double p = B * std::pow(totalInvest, a) / (std::pow(q, a) + std::pow(totalInvest, a));
        const double share = p / n;
        const double g = std::pow(share, a) / (1 + std::pow(share, a));
        res = g - c * invest;
    }
    else
    {
        res = exp(-std::pow(totalInvest - 0.5, 2) / 0.05);
    }
    return res;
}


void PartnerControlWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}

void PartnerControlWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<PartnerControlController*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void PartnerControlWorldObserver::activateOnlyRobot(int robotIndex)
{
    auto *ctl = dynamic_cast<PartnerControlController *>(m_world->getRobot(0)->getController());
    ctl->loadNewGenome(m_individuals[robotIndex]);
}

