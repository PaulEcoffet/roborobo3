//
// Created by paul on 06/11/17.
//

#include <core/RoboroboMain/main.h>
#include <CorrectRepartition/include/CorrectRepartitionOpportunity.h>
#include <CorrectRepartition/include/CorrectRepartitionSharedData.h>
#include <CorrectRepartition/include/CorrectRepartitionController.h>
#include <CorrectRepartition/include/CorrectRepartitionWorldModel.h>
#include "CorrectRepartition/include/CorrectRepartitionAnalysisWorldObserver.h"
#include "contrib/json/json.hpp"

using json = nlohmann::json;

CorrectRepartitionAnalysisWorldObserver::CorrectRepartitionAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    CorrectRepartitionSharedData::initSharedData();

    if (gInitialNumberOfRobots != 1)
    {
        std::cerr << "Only one robot must be instanced for this run";
        exit(-1);
    }

    std::string genomePath(gProperties.getProperty("genomePath"));
    if (!boost::filesystem::exists(genomePath))
    {
        std::cerr << "The genome file path '" << genomePath << "' does not exist.\n";
        exit(-1);
    }
    std::ifstream genomeFile(genomePath);
    genomeFile >> m_genomesJson;
    m_genomesIt = m_genomesJson.begin();

    std::cout << gLogDirectoryname + "/analysis_log.txt";
    m_log.open(gLogDirectoryname + "/analysis_log.txt");
    m_log << "ind\tcoop\trep\tit\tonOpp\townInv\n";


    gProperties.checkAndGetPropertyValue("analysisIterationPerRep", &m_nbIterationPerRep, true);
    gProperties.checkAndGetPropertyValue("analysisNbRep", &m_nbRep, true);
    m_curCoop = 0;
    m_curInd = 0;
    m_curIterationInRep = 0;
    m_curRep = 0;

    gMaxIt = -1;
}

void CorrectRepartitionAnalysisWorldObserver::reset()
{
    loadGenome((*m_genomesIt));
    resetEnvironment();
}

void CorrectRepartitionAnalysisWorldObserver::stepPre()
{
    computeOpportunityImpact();
    monitorPopulation();
    clearOpportunityNearbyRobots();
    m_curIterationInRep++;
    if (m_curIterationInRep == m_nbIterationPerRep)
    {
        this->logRep();
        this->resetEnvironment();
        m_curIterationInRep = 0;
        m_onOppTotal = 0;
        m_curRep++;

    }
    if (m_curRep == m_nbRep)
    {
        m_curCoop += m_stepCoop;
        this->setAllOpportunitiesCoop(m_curCoop);
        m_curRep = 0;
    }
    /* commented out as useless for now.
    if (m_curCoop > CorrectRepartitionSharedData::maxCoop)
    {
        m_curCoop = 0;
        this->setAllOpportunitiesCoop(m_curCoop);
        m_genomesIt++;
        if (m_genomesIt < m_genomesJson.end())
        {
            m_curInd++;
            m_log << std::flush;
            loadGenome((*m_genomesIt));
        }
        else
        {
            std::cout << "Over" << "\n";
            m_log.close();
            exit(0);
        }
    }*/
}

void CorrectRepartitionAnalysisWorldObserver::monitorPopulation()
{
    /*
    auto *wm = dynamic_cast<CorrectRepartitionWorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;
    out << m_curInd << "\t";
    out << m_curCoop << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << (int) wm->onOpportunity << "\t";
    out << wm->_cooperationLevel << "\n";
    m_log << out.str();
    */
}

void CorrectRepartitionAnalysisWorldObserver::computeOpportunityImpact()
{
    /*
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < _world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<CorrectRepartitionWorldModel*>(_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<CorrectRepartitionWorldModel*>(_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<CorrectRepartitionController*>(_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            ctl->increaseFitness(opp->getCoop()  + wm->_cooperationLevel);
        }
    }
     */
}

void CorrectRepartitionAnalysisWorldObserver::resetEnvironment()
{
    /*
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }
    setAllOpportunitiesCoop(m_curCoop);

    for (auto object: gPhysicalObjects)
    {
        object->resetLocation();
        object->registerObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
    }
     */
}

void CorrectRepartitionAnalysisWorldObserver::loadGenome(const std::vector<double> &weights)
{
    auto *ctl = dynamic_cast<CorrectRepartitionController *>(gWorld->getRobot(0)->getController());
    ctl->loadNewGenome(weights);
    ctl->resetFitness();
}

void CorrectRepartitionAnalysisWorldObserver::setAllOpportunitiesCoop(const double coop)
{
    /*
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(phys);
        opp->setCoopValue(coop);
    }
     */
}


void CorrectRepartitionAnalysisWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CorrectRepartitionOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}

void CorrectRepartitionAnalysisWorldObserver::logRep()
{

}
