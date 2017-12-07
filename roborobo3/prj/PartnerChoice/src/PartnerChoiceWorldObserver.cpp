//
// Created by paul on 30/10/17.
//

#include <PartnerChoice/include/PartnerChoiceWorldModel.h>
#include <core/Utilities/Graphics.h>
#include <PartnerChoice/include/PartnerChoiceAnalysisWorldObserver.h>
#include <PartnerChoice/include/PartnerChoiceWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "core/RoboroboMain/main.h"
#include "PartnerChoice/include/PartnerChoiceWorldObserver.h"
#include "PartnerChoice/include/PartnerChoiceController.h"
#include "PartnerChoice/include/PartnerChoiceSharedData.h"


PartnerChoiceWorldObserver::PartnerChoiceWorldObserver(World *__world) : WorldObserver(__world)
{
    m_world = __world;

    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;
    m_curInd = 0;

    PartnerChoiceSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");
    std::cout << "gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n";

    m_observer = new LogManager(gLogDirectoryname + "/observer.txt");
    m_observer->write("gen\tind\teval\titer\tonOpp\tcoop\n");

    m_nbIndividuals = 50;
    gMaxIt = -1;

    std::vector<std::string> url;
    if (gRemote == "")
    {
        std::cerr << "[WARNING] gRemote needs to be defined.";
        url.emplace_back("127.0.0.1");
        url.emplace_back("1703");
    }
    boost::split(url, gRemote, boost::is_any_of(":"));
    pycma.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));
}


PartnerChoiceWorldObserver::~PartnerChoiceWorldObserver()
{
    delete m_fitnessLogManager;
    delete m_observer;
}

void PartnerChoiceWorldObserver::reset()
{
    // Init environment
    initOpportunities();

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<PartnerChoiceController * >(m_world->getRobot(0)->getController())->getGenomeSize();
    m_individuals = pycma.initCMA(m_nbIndividuals, nbweights);
    m_fitnesses.resize(m_nbIndividuals, 0);

    activateOnlyRobot(m_curInd);
    resetEnvironment();
}

void PartnerChoiceWorldObserver::initOpportunities()
{
    double curCoopVal = 0;
    double stepCoop = 0;
    if (PartnerChoiceSharedData::nbCoopStep > 1)
    {
        stepCoop = (PartnerChoiceSharedData::maxCoop - curCoopVal) / ((double)PartnerChoiceSharedData::nbCoopStep - 1);
    }
    else
    {
        curCoopVal = PartnerChoiceSharedData::maxCoop;
    }
    int stepEvery = std::ceil(gNbOfPhysicalObjects / PartnerChoiceSharedData::nbCoopStep);
    int i = 0;
    auto physObjList = gPhysicalObjects;
    std::shuffle(physObjList.begin(), physObjList.end(), engine);
    for (auto *physicalObject : physObjList)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        opp->setCoopValue(curCoopVal);
        if ((i + 1) == stepEvery)
        {
            curCoopVal += stepCoop;
            i = 0;
        }
        else
        {
            i++;
        }

    }
}

void PartnerChoiceWorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == PartnerChoiceSharedData::evaluationTime)
    {
        resetEnvironment();
        m_curEvaluationIteration = 0;
        m_curEvaluationInGeneration++;
    }
    if( m_curEvaluationInGeneration == PartnerChoiceSharedData::nbEvaluationsPerGeneration)
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

void PartnerChoiceWorldObserver::stepPost()
{
    computeOpportunityImpact();
    monitorPopulation();
    clearOpportunityNearbyRobots();
}

void PartnerChoiceWorldObserver::monitorPopulation() const
{
    if ((m_generationCount + 1) % PartnerChoiceSharedData::takeVideoEveryGeneration == 0 && m_curInd % 25 == 0)
    {
        std::string name = "gen_" + std::to_string(m_generationCount) + "_ind_" + std::to_string(m_curInd);
        saveCustomScreenshot(name);
    }

    if ((m_generationCount + 1) % PartnerChoiceSharedData::takeVideoEveryGeneration == 0)
    {
        std::stringstream out;
        auto *wm = dynamic_cast<PartnerChoiceWorldModel *>(m_world->getRobot(0)->getWorldModel());
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

void PartnerChoiceWorldObserver::stepEvolution()
{
    logFitnesses(getSortedFitnesses());
    m_individuals = pycma.getNextGeneration(m_fitnesses);
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> PartnerChoiceWorldObserver::getSortedFitnesses() const
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

void PartnerChoiceWorldObserver::logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses)
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
}


void PartnerChoiceWorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }
    int i = 0;
    for (auto object: gPhysicalObjects)
    {
        object->findRandomLocation();
        object->registerObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->registerRobot();
    }
}

void PartnerChoiceWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<PartnerChoiceWorldModel*>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<PartnerChoiceWorldModel*>(m_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<PartnerChoiceController*>(m_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(0);
            wm->appendTotalInvest(opp->getCoop());

            //Reward him
            ctl->increaseFitness(opp->getCoop());
        }
    }
}

double PartnerChoiceWorldObserver::payoff(const double invest, const double totalInvest) const
{
    const int nbRobots = 2;
    const double coeff = PartnerChoiceSharedData::constantK / (1.0 + pow(nbRobots - 2, 2)); // \frac{k}{1+(n-2)^2}
    const double res = coeff * pow(totalInvest, PartnerChoiceSharedData::constantA) - invest;
    return res;
}


void PartnerChoiceWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}


void PartnerChoiceWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<PartnerChoiceController*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void PartnerChoiceWorldObserver::activateOnlyRobot(int robotIndex)
{
    auto *ctl = dynamic_cast<PartnerChoiceController *>(m_world->getRobot(0)->getController());
    ctl->loadNewGenome(m_individuals[robotIndex]);
}

