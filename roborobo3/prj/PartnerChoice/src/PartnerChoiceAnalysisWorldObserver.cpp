//
// Created by paul on 06/11/17.
//

#include <RoboroboMain/main.h>
#include <PartnerChoice/include/PartnerChoiceOpportunity.h>
#include <PartnerChoice/include/PartnerChoiceSharedData.h>
#include <PartnerChoice/include/PartnerChoiceController.h>
#include <PartnerChoice/include/PartnerChoiceWorldModel.h>
#include "PartnerChoice/include/PartnerChoiceAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

PartnerChoiceAnalysisWorldObserver::PartnerChoiceAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    PartnerChoiceSharedData::initSharedData();

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
    m_stepCoop = PartnerChoiceSharedData::maxCoop / ((double) PartnerChoiceSharedData::nbCoopStep - 1);
    m_curCoop = 0;
    m_curInd = 0;
    m_curIterationInRep = 0;
    m_curRep = 0;

    gMaxIt = m_genomesJson.size() * m_nbIterationPerRep * m_nbRep * PartnerChoiceSharedData::nbCoopStep;
}

void PartnerChoiceAnalysisWorldObserver::reset()
{
    loadGenome((*m_genomesIt));
    resetEnvironment();
}

void PartnerChoiceAnalysisWorldObserver::stepPre()
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
    if (m_curCoop > PartnerChoiceSharedData::maxCoop)
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
    }
}

void PartnerChoiceAnalysisWorldObserver::monitorPopulation()
{
    auto *wm = dynamic_cast<PartnerChoiceWorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;
    out << m_curInd << "\t";
    out << m_curCoop << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << (int) wm->onOpportunity << "\t";
    out << wm->_cooperationLevel << "\n";
    m_log << out.str();
}

void PartnerChoiceAnalysisWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < _world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<PartnerChoiceWorldModel *>(_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<PartnerChoiceWorldModel *>(_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<PartnerChoiceController *>(_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            ctl->increaseFitness(opp->getCoop() + wm->_cooperationLevel);
        }
    }
}

void PartnerChoiceAnalysisWorldObserver::resetEnvironment()
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
    setAllOpportunitiesCoop(m_curCoop);

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

void PartnerChoiceAnalysisWorldObserver::loadGenome(const std::vector<double> &weights)
{
    auto *ctl = dynamic_cast<PartnerChoiceController *>(gWorld->getRobot(0)->getController());
    ctl->loadNewGenome(weights);
    ctl->resetFitness();
}

void PartnerChoiceAnalysisWorldObserver::setAllOpportunitiesCoop(const double coop)
{
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(phys);
        opp->setCoopValue(coop);
    }
}


void PartnerChoiceAnalysisWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}

void PartnerChoiceAnalysisWorldObserver::logRep()
{

}
