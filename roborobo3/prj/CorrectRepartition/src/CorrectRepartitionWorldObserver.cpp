//
// Created by paul on 30/10/17.
//

#include <CorrectRepartition/include/CorrectRepartitionWorldModel.h>
#include <Utilities/Graphics.h>
#include <CorrectRepartition/include/CorrectRepartitionAnalysisWorldObserver.h>
#include <CorrectRepartition/include/CorrectRepartitionWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "RoboroboMain/main.h"
#include "CorrectRepartition/include/CorrectRepartitionWorldObserver.h"
#include "CorrectRepartition/include/CorrectRepartitionController.h"
#include "CorrectRepartition/include/CorrectRepartitionSharedData.h"


CorrectRepartitionWorldObserver::CorrectRepartitionWorldObserver(World *__world) : WorldObserver(__world)
{
    m_world = __world;

    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;
    m_curBatch = 0;

    CorrectRepartitionSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tind\trep\tfit\n");

    m_observer = new LogManager(gLogDirectoryname + "/observer.txt");
    m_observer->write("gen\tbatch\teval\titer\toppid\tagentid\n");

    m_nbIndividuals = 40;
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


CorrectRepartitionWorldObserver::~CorrectRepartitionWorldObserver()
{
    delete m_fitnessLogManager;
    delete m_observer;
}

void CorrectRepartitionWorldObserver::reset()
{
    // Init fitness
    clearRobotFitnesses();

    if (CorrectRepartitionSharedData::clones)
    {
        m_batchSize = 1;
    }
    else
    {
        m_batchSize = gNbOfRobots;
    }

    assert(m_nbIndividuals % m_batchSize == 0);


    // create individuals
    int nbweights = dynamic_cast<CorrectRepartitionController * >(m_world->getRobot(
            0)->getController())->getGenomeSize();
    m_individuals = pycma.initCMA(m_nbIndividuals, nbweights);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_shuffledIndividualId.resize(m_nbIndividuals, 0);
    std::iota(m_shuffledIndividualId.begin(), m_shuffledIndividualId.end(), 0); // vector of 0, 1, .., m_nbIndividuals
    if (!CorrectRepartitionSharedData::clones)
    { // Don't shuffle if it's with clones. Make less weird fitness logs
        std::shuffle(m_shuffledIndividualId.begin(), m_shuffledIndividualId.end(), engine); // engine : Mersenne twister
    }
    activateOnlyRobot(m_curBatch);
    resetEnvironment();
}


void CorrectRepartitionWorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == CorrectRepartitionSharedData::evaluationTime)
    {
        resetEnvironment();
        m_curEvaluationIteration = 0;
        std::stringstream out;

        for (int i = 0; i < gNbOfRobots; i++)
        {
            int curRobot = 0;
            if (CorrectRepartitionSharedData::clones)
            {
                curRobot = m_curBatch;
            }
            else
            {
                curRobot = m_curBatch * m_batchSize + i;
            }
            auto *wm = m_world->getRobot(i)->getWorldModel();
            out << m_generationCount << "\t" << m_shuffledIndividualId[curRobot] << "\t" << m_curEvaluationInGeneration
                << "\t"
                << wm->_fitnessValue << "\n";
            m_fitnesses[m_shuffledIndividualId[curRobot]] += wm->_fitnessValue;
        }
        m_fitnessLogManager->write(out.str());

        clearRobotFitnesses();
        m_curBatch++;

        if (m_curBatch < m_nbIndividuals / m_batchSize)
        {
            activateOnlyRobot(m_curBatch);
        }
    }
    if (m_curBatch == m_nbIndividuals / m_batchSize)
    {
        m_curEvaluationInGeneration++;
        m_curBatch = 0;
        // Shuffle the pairing for the new evaluation
        if (!CorrectRepartitionSharedData::clones)
        { // Don't shuffle if it's with clones. Make less weird fitness logs
            std::shuffle(m_shuffledIndividualId.begin(), m_shuffledIndividualId.end(), engine); // engine: global var
        }
    }
    if (m_curEvaluationInGeneration == CorrectRepartitionSharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_curBatch = 0;
        activateOnlyRobot(m_curBatch);
        m_generationCount++;
    }
}

void CorrectRepartitionWorldObserver::stepPost()
{
    computeOpportunityImpact();
    monitorPopulation();
    clearOpportunityNearbyRobots();
}

void CorrectRepartitionWorldObserver::monitorPopulation() const
{
    if ((m_generationCount + 1) % CorrectRepartitionSharedData::takeVideoEveryGeneration == 0 && m_curBatch % 25 == 0)
    {
        std::string name = "gen_" + std::to_string(m_generationCount) + "_ind_" + std::to_string(m_curBatch);
        saveCustomScreenshot(name);
    }

    if ((m_generationCount + 1) % CorrectRepartitionSharedData::takeVideoEveryGeneration == 0)
    {
        std::stringstream out;
        for (int i = 0; i < gNbOfPhysicalObjects; i++)
        {
            auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(gPhysicalObjects[i]);
            for (auto rid : opp->getNearbyRobotIndexes())
            {
                out << m_generationCount << "\t";
                out << m_curBatch << "\t";
                out << m_curEvaluationInGeneration << "\t";
                out << i << "\t";
                out << m_curEvaluationIteration << "\t";
                out << rid << "\n";
            }
        }
        m_observer->write(out.str());

    }
}

void CorrectRepartitionWorldObserver::stepEvolution()
{
    if ((m_generationCount + 1) % CorrectRepartitionSharedData::genomeLog == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << std::setw(2) << json(m_individuals);
    }
    m_individuals = pycma.getNextGeneration(m_individuals, m_fitnesses);
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
}


void CorrectRepartitionWorldObserver::resetEnvironment()
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
    for (auto object: gPhysicalObjects)
    {
        object->findRandomLocation();
        object->registerObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->registerRobot();
    }
}

void CorrectRepartitionWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<CorrectRepartitionWorldModel *>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->nbOnOpp = 0;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(physicalObject);
        opp->registerNewNearbyRobots();
        int arrival = 0;
        for (auto index : opp->getNearbyRobotIndexes())
        {
            arrival += 1;
            auto *wm = dynamic_cast<CorrectRepartitionWorldModel *>(m_world->getRobot(index)->getWorldModel());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;
            if (CorrectRepartitionSharedData::arrivalMemory)
            {
                wm->nbOnOpp = arrival;
            }
            else
            {
                wm->nbOnOpp = opp->getNbNearbyRobots();
            }

            // Add information about his previous investment
            // wm->appendOwnInvest(wm->_cooperationLevel);
            // wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            if (!CorrectRepartitionSharedData::ifThreeNoGain || arrival < 3)
                wm->_fitnessValue += payoff(opp->getNbNearbyRobots());
        }
    }
}

double CorrectRepartitionWorldObserver::payoff(const int nbRobots) const
{
    double res = 1.0 / (1.0 + (nbRobots - 2) * (nbRobots - 2));
    if (CorrectRepartitionSharedData::exactlyTwo)
    {
        if (nbRobots == 2)
            res = 1;
        else
            res = 0;
    }
    return res;
}


void CorrectRepartitionWorldObserver::clearOpportunityNearbyRobots()
{
    /*
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
     */
}


void CorrectRepartitionWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        auto *ctl = dynamic_cast<CorrectRepartitionController *>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void CorrectRepartitionWorldObserver::activateOnlyRobot(int batchIndex)
{
    for (int i = 0; i < gNbOfRobots; i++)
    {
        int robId = 0;

        if (CorrectRepartitionSharedData::clones)
        {
            robId = batchIndex;
        }
        else
        {
            robId = batchIndex * m_batchSize + i;
        }
        auto *ctl = dynamic_cast<CorrectRepartitionController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(m_individuals[m_shuffledIndividualId[robId]]);
    }
}

