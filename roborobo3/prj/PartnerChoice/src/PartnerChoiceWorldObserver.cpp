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
        auto *wm = m_world->getRobot(0)->getWorldModel();
        std::stringstream out;
        out << m_generationCount << "\t" << m_curInd << "\t" << m_curEvaluationInGeneration << "\t"
            << wm->_fitnessValue << "\n";
        m_fitnessLogManager->write(out.str());
        m_fitnesses[m_curInd] += wm->_fitnessValue;
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;
    }
    if( m_curEvaluationInGeneration == PartnerChoiceSharedData::nbEvaluationsPerGeneration)
    {
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
    if ((m_generationCount+1) % PartnerChoiceSharedData::genomeLog == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    m_individuals = pycma.getNextGeneration(m_fitnesses);
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
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

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            wm->_fitnessValue += payoff(wm->_cooperationLevel, opp->getCoop() + wm->_cooperationLevel);
        }
    }
}

double PartnerChoiceWorldObserver::payoff(const double invest, const double totalInvest) const
{
    const double n = 2, c = 0.5;
    double res = (totalInvest / n) - (0.5 * c * invest * invest);
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

