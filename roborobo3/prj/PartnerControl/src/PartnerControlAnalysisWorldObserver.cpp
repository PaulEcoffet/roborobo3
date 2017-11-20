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

    std::ifstream genomeFile(gLogDirectoryname + "/genome.txt");
    genomeFile >> m_genomesJson;
    m_genomesIt = m_genomesJson.begin();
    loadGenome((*m_genomesIt)["weights"]);

    gProperties.checkAndGetPropertyValue("analysisIterationPerRep", &m_nbIterationPerRep, true);
    gProperties.checkAndGetPropertyValue("analysisNbRep", &m_nbRep, true);
    m_stepCoop = PartnerControlSharedData::maxCoop / ((double)PartnerControlSharedData::nbCoopStep - 1);
    gMaxIt = static_cast<int>(m_genomesJson.size() * m_nbIterationPerRep * m_nbRep * PartnerControlSharedData::nbCoopStep);
}

void PartnerControlAnalysisWorldObserver::reset()
{
    // deactivate all the robots except one
    resetEnvironment();

}

void PartnerControlAnalysisWorldObserver::step()
{
    monitorPopulation();
    m_curIterationInRep++;
    if (m_curIterationInRep == m_nbIterationPerRep)
    {
        this->resetEnvironment();
        m_curIterationInRep = 0;
        m_curRep++;
    }
    if (m_curRep == m_nbRep)
    {
        m_curCoop += m_stepCoop;
        this->setAllOpportunitiesCoop(m_curCoop);
        m_curRep = 0;
    }
    if (m_curCoop > PartnerControlSharedData::maxCoop)
    {
        m_curCoop = 0;
        m_genomesIt++;
        this->loadGenome((*m_genomesIt)["weights"]);
    }
}

void PartnerControlAnalysisWorldObserver::monitorPopulation()
{
    auto *wm = dynamic_cast<PartnerControlWorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;
    out << (*m_genomesIt)["generation"] << "\t";
    out << (*m_genomesIt)["ind"] << "\t";
    out << (*m_genomesIt)["rank"] << "\t";
    out << m_curCoop << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << wm->onOpportunity << "\n";
    out.str();
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
    PartnerControlController::genome genome;
    genome.sigma  = 0;
    genome.weights = weights;
    ctl->loadNewGenome(genome);
}

void PartnerControlAnalysisWorldObserver::setAllOpportunitiesCoop(const double coop)
{
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<PartnerControlOpportunity *>(phys);
        opp->setCoopValue(coop);
    }
}
