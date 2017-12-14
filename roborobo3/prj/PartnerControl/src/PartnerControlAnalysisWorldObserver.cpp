//
// Created by paul on 06/11/17.
//

#include <core/RoboroboMain/main.h>
#include <PartnerControl/include/PartnerControlOpportunity.h>
#include <PartnerControl/include/PartnerControlSharedData.h>
#include <PartnerControl/include/PartnerControlController.h>
#include <PartnerControl/include/PartnerControlWorldModel.h>
#include "PartnerControl/include/PartnerControlAnalysisWorldObserver.h"
#include "contrib/json/json.hpp"

using json = nlohmann::json;

PartnerControlAnalysisWorldObserver::PartnerControlAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    PartnerControlSharedData::initSharedData();

    if (gInitialNumberOfRobots != 1)
    {
        std::cerr << "Only one robot must be instanced for this run";
        exit(-1);
    }

    std::string genomePath(gLogDirectoryname + "/genome.txt");
    gProperties.checkAndGetPropertyValue("genomePath", &genomePath, false);
    if (!boost::filesystem::exists(genomePath))
    {
        std::cerr << "The genome file path '" << genomePath << "' does not exist.\n";
        exit(-1);
    }
    std::ifstream genomeFile(genomePath);
    genomeFile >> m_genomesJson;
    m_genomesIt = m_genomesJson.begin();
    
    m_log.open(gLogDirectoryname + "/analysis_log.txt");
    m_log << "generation\tind\trank\tcoop\trep\tit\tonOpp\townCoop\n";


    gProperties.checkAndGetPropertyValue("analysisIterationPerRep", &m_nbIterationPerRep, true);
    gProperties.checkAndGetPropertyValue("analysisNbRep", &m_nbRep, true);
    m_stepCoop = PartnerControlSharedData::maxCoop / ((double)PartnerControlSharedData::nbCoopStep - 1);
    m_curCoop = 0;
    m_curIterationInRep = 0;
    m_curRep = 0;

    gMaxIt = m_genomesJson.size() * m_nbIterationPerRep * m_nbRep * PartnerControlSharedData::nbCoopStep;
}

void PartnerControlAnalysisWorldObserver::reset()
{
    loadGenome((*m_genomesIt)["weights"]);
    resetEnvironment();
}

void PartnerControlAnalysisWorldObserver::stepPre()
{
    computeOpportunityImpact();
    monitorPopulation();
    clearOpportunityNearbyRobots();
    m_curIterationInRep++;
    if (m_curIterationInRep == m_nbIterationPerRep)
    {
        this->resetEnvironment();
        m_curIterationInRep = 0;
        m_curRep++;

    }
    if (m_curRep == m_nbRep)
    {
        std::cout << "coop: " << m_curCoop << ", ind: " << (*m_genomesIt)["id"]
                  << ", rank: " << (*m_genomesIt)["rank"] << ", gen: " << (*m_genomesIt)["generation"] << "\n";
        m_curCoop += m_stepCoop;
        this->setAllOpportunitiesCoop(m_curCoop);
        m_curRep = 0;
    }
    if (m_curCoop > PartnerControlSharedData::maxCoop + 0.1)
    {
        m_curCoop = 0;
        this->setAllOpportunitiesCoop(m_curCoop);
        m_genomesIt++;
        if (m_genomesIt < m_genomesJson.end())
        {
            m_log << std::flush;
            loadGenome((*m_genomesIt)["weights"]);
        }
        else
        {
            std::cout << "Over" << "\n";
            m_log.close();
            exit(0);
        }
    }
}

void PartnerControlAnalysisWorldObserver::monitorPopulation()
{
    auto *wm = dynamic_cast<PartnerControlWorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;
    out << (*m_genomesIt)["generation"] << "\t";
    out << (*m_genomesIt)["id"] << "\t";
    out << (*m_genomesIt)["rank"] << "\t";
    out << m_curCoop << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << (int) wm->onOpportunity << "\t";
    out << wm->_cooperationLevel << "\n";
    m_log << out.str();
}


void PartnerControlAnalysisWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < _world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<PartnerControlWorldModel*>(_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(physicalObject);
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<PartnerControlWorldModel*>(_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<PartnerControlController*>(_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);

            //Reward him
            ctl->increaseFitness(0);
        }
    }
}

void PartnerControlAnalysisWorldObserver::resetEnvironment()
{
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
}

void PartnerControlAnalysisWorldObserver::loadGenome(const std::vector<double> &weights)
{
    auto *ctl = dynamic_cast<PartnerControlController *>(gWorld->getRobot(0)->getController());
    ctl->loadNewGenome(weights);
    ctl->resetFitness();
}

void PartnerControlAnalysisWorldObserver::setAllOpportunitiesCoop(const double coop)
{
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(phys);
        opp->setCoopValue(coop);
    }
}


void PartnerControlAnalysisWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}
