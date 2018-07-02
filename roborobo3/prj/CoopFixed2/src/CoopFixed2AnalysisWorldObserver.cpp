/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "CoopFixed2/include/CoopFixed2AnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <CoopFixed2/include/CoopFixed2Opportunity.h>
#include <CoopFixed2/include/CoopFixed2SharedData.h>
#include <CoopFixed2/include/CoopFixed2Controller.h>
#include <CoopFixed2/include/CoopFixed2WorldModel.h>
#include <CoopFixed2/include/CoopFixed2AnalysisOpportunity.h>
#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include "CoopFixed2/include/CoopFixed2AnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

CoopFixed2AnalysisWorldObserver::CoopFixed2AnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    CoopFixed2SharedData::initSharedData();

    if (gInitialNumberOfRobots != 1)
    {
        std::cerr << "Only one robot must be instanced for this run";
        exit(-1);
    }

    int gen = 0;
    gProperties.checkAndGetPropertyValue("genAnalysis", &gen, true);

    std::string genomePath(gLogDirectoryname + "/genomes_" + std::to_string(gen) + ".txt");
    gProperties.checkAndGetPropertyValue("genomePath", &genomePath, false);
    if (!boost::filesystem::exists(genomePath))
    {
        std::cerr << "The genome file path '" << genomePath << "' does not exist.\n";
        exit(-1);
    }
    std::ifstream genomeFile(genomePath);
    genomeFile >> m_genomesJson;
    m_genomesIt = m_genomesJson.begin();

    m_log.open(gLogDirectoryname + "/analysis_log_" + std::to_string(gen) + ".txt");
    m_log << "ind\trep\tit\toppId\toppCoop\toppNb\townCoop\n";


    gProperties.checkAndGetPropertyValue("analysisIterationPerRep", &m_nbIterationPerRep, true);
    gProperties.checkAndGetPropertyValue("analysisNbRep", &m_nbRep, true);
    m_curIterationInRep = 0;
    m_curRep = 0;
    m_curInd = 0;

    gMaxIt = -1;
}

void CoopFixed2AnalysisWorldObserver::reset()
{
    loadGenome((*m_genomesIt));
    resetEnvironment();
}

void CoopFixed2AnalysisWorldObserver::stepPre()
{
    const double maxCoop = CoopFixed2SharedData::maxCoop;
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
        m_curCoop += m_stepCoop;
        m_curRep = 0;
        m_genomesIt++;
        m_curInd++;
        if (m_genomesIt < m_genomesJson.end())
        {
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

void CoopFixed2AnalysisWorldObserver::stepPost()
{
    if (CoopFixed2SharedData::tpToNewObj)
    {
        auto randomPhys = std::uniform_int_distribution<int>(0, gNbOfPhysicalObjects - 1);
        for (int i = 0; i < gWorld->getNbOfRobots(); i++)
        {
            auto *rob = gWorld->getRobot(i);
            if (dynamic_cast<CoopFixed2WorldModel* >(rob->getWorldModel())->teleport)
            {
                int dest_obj = randomPhys(engine);
                PhysicalObject *physobj = gPhysicalObjects[dest_obj];
                rob->unregisterRobot();
                rob->setCoord(physobj->getXCenterPixel(), physobj->getYCenterPixel());
                rob->setCoordReal(physobj->getXCenterPixel(), physobj->getYCenterPixel());
                rob->registerRobot();
            }
        }
    }


    for (auto id: objectsToTeleport)
    {
        gPhysicalObjects[id]->unregisterObject();
        gPhysicalObjects[id]->resetLocation();
        dynamic_cast<CoopFixed2Opportunity *>(gPhysicalObjects[id])->resetLife();
        gPhysicalObjects[id]->registerObject();
    }
    objectsToTeleport.clear();

    computeOpportunityImpact();
    monitorPopulation();
}

void CoopFixed2AnalysisWorldObserver::monitorPopulation()
{
    auto *wm = dynamic_cast<CoopFixed2WorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;

    int oppLifeId = 0;
    double oppCoop = 0;
    int oppNbRob = 0;
    if (wm->opp != nullptr) {
        oppLifeId = wm->opp->lifeid;
        oppCoop = dynamic_cast<CoopFixed2AnalysisOpportunity*>(wm->opp)->getCoop();
        oppNbRob = dynamic_cast<CoopFixed2AnalysisOpportunity *>(wm->opp)->getNbFakeRobots();
    }

    out << m_curInd << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << oppLifeId << "\t";
    out << oppCoop << "\t";
    out << oppNbRob << "\t";
    out << wm->_cooperationLevel << "\n";
    m_log << out.str();
}


void CoopFixed2AnalysisWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < _world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<CoopFixed2WorldModel*>(_world->getRobot(i)->getWorldModel());
        wm->opp = nullptr;
        wm->onOpportunity = false;
        wm->arrival = 0;
        wm->nbOnOpp = 0;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CoopFixed2AnalysisOpportunity *>(physicalObject);
        opp->curInv = opp->getCoop();
        for (auto index : opp->getNearbyRobotIndexes()) {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<CoopFixed2Controller *>(_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;
            wm->opp = opp;
            wm->arrival = 1;
            wm->nbOnOpp = opp->getNbNearbyRobots();

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            if (CoopFixed2SharedData::onlyOtherInTotalInv)
            {
                wm->appendTotalInvest(opp->getCoop());
            } else {
                wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);
            }
            //Reward him
            ctl->increaseFitness(0);
            opp->curInv = opp->getCoop() + wm->_cooperationLevel;
        }
    }
}

void CoopFixed2AnalysisWorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }

    int i = 0;
    int objPerCoop = (int)ceil((double)gNbOfPhysicalObjects / 12); // coop : 0, 1, .., 10 + obj without ind
    int nbRob = 0;
    int coop = 0;
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CoopFixed2AnalysisOpportunity *>(phys);
        opp->setCoopValue(coop);
        opp->setNbFakeRobots(nbRob);
        i++;
        if (i % objPerCoop == 0)
        {
            if (nbRob == 0) {
                nbRob = 1;
            } else {
                coop += CoopFixed2SharedData::maxCoop / 10;
            }
        }
        phys->resetLocation();
        phys->registerObject();
    }


    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        if (CoopFixed2SharedData::tpToNewObj)
        {
            robot->getWorldModel()->_agentAbsoluteOrientation = 0;
        }
        dynamic_cast<CoopFixed2WorldModel*>(robot->getWorldModel())->reset();
    }
}

void CoopFixed2AnalysisWorldObserver::loadGenome(const std::vector<double> &weights)
{
    auto *ctl = dynamic_cast<CoopFixed2Controller *>(gWorld->getRobot(0)->getController());
    ctl->loadNewGenome(weights);
    ctl->resetFitness();
}


void CoopFixed2AnalysisWorldObserver::clearOpportunityNearbyRobots()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CoopFixed2AnalysisOpportunity *>(physicalObject);
        opp->clearNearbyRobotIndexes();
    }
}


void CoopFixed2AnalysisWorldObserver::addObjectToTeleport(int id) {
    objectsToTeleport.emplace(id);
}