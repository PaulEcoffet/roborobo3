/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "Negociate/include/NegociateAnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <Negociate/include/NegociateOpportunity.h>
#include <Negociate/include/NegociateSharedData.h>
#include <Negociate/include/NegociateController.h>
#include <Negociate/include/NegociateWorldModel.h>
#include <Negociate/include/NegociateAnalysisOpportunity.h>
#include <Negociate/include/NegociateWorldObserver.h>
#include "Negociate/include/NegociateAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

NegociateAnalysisWorldObserver::NegociateAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    NegociateSharedData::initSharedData();

    if (gInitialNumberOfRobots != 1)
    {
        std::cerr << "Only one robot must be instanced for this run\n";
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
    m_log << "ind\trep\tit\toppId\toppCoop\toppNb\townCoop\tspite\tteleport\n";


    gProperties.checkAndGetPropertyValue("analysisIterationPerRep", &m_nbIterationPerRep, true);
    gProperties.checkAndGetPropertyValue("analysisNbRep", &m_nbRep, true);
    //gProperties.checkAndGetPropertyValue("maxrobnb", &m_maxrobnb, true);

    m_maxrobnb = 50;
    m_curIterationInRep = 0;
    m_curRep = 0;
    m_curInd = 0;
    m_curCoop = 0;
    m_curnbrob = 0;
    m_stepCoop = 0.1;

    gMaxIt = -1;
}

void NegociateAnalysisWorldObserver::reset()
{
    loadGenome((*m_genomesIt));
    initObjects();
    resetEnvironment();
}

void NegociateAnalysisWorldObserver::stepPre()
{
    const double maxCoop = NegociateSharedData::maxCoop;
    m_curIterationInRep++;
    if (m_curIterationInRep == m_nbIterationPerRep)
    {
        resetEnvironment();
        m_curIterationInRep = 0;
        m_curRep++;

    }
    if (m_curRep == m_nbRep) {
        m_curCoop += m_stepCoop;
        m_curRep = 0;
        resetEnvironment();
    }
    if (m_curCoop > maxCoop)
    {
        m_curCoop = 0;
        if (m_curnbrob < 10)
        {
            m_curnbrob += 1;
        }
        else {
            m_curnbrob += 10;
        }
        if (m_curnbrob <= m_maxrobnb)
            resetEnvironment();
    }
    if (m_curnbrob > m_maxrobnb)
    {
        m_curnbrob = 0;
        m_genomesIt++;
        m_curInd++;
        if (m_genomesIt < m_genomesJson.end())
        {
            m_log << std::flush;
            loadGenome((*m_genomesIt));
            resetEnvironment();
        }
        else
        {
            std::cout << "Over\n";
            m_log.close();
            exit(0);
        }
    }
}

void NegociateAnalysisWorldObserver::stepPost()
{
    registerRobotsOnOpportunities();
    computeOpportunityImpact();
    monitorPopulation();
}

void NegociateAnalysisWorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<NegociateOpportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void NegociateAnalysisWorldObserver::monitorPopulation()
{
    auto *wm = dynamic_cast<NegociateWorldModel *>(gWorld->getRobot(0)->getWorldModel());
    std::stringstream out;

    int oppLifeId = -1;
    if (wm->opp != nullptr)
    {
        oppLifeId = wm->opp->lifeid;
    }

    out << m_curInd << "\t";
    out << m_curRep << "\t";
    out << m_curIterationInRep << "\t";
    out << oppLifeId << "\t";
    out << m_curCoop << "\t";
    out << m_curnbrob << "\t";
    out << wm->_cooperationLevel << "\t";
    out << wm->spite << "\t";
    out << wm->teleport << "\n";
    m_log << out.str();
}

void NegociateAnalysisWorldObserver::computeOpportunityImpact()
{
    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < _world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<NegociateWorldModel *>(_world->getRobot(i)->getWorldModel());
        wm->opp = nullptr;
        wm->onOpportunity = false;
        wm->arrival = 0;
        wm->nbOnOpp = 0;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<NegociateAnalysisOpportunity *>(physicalObject);
        opp->curInv = opp->getCoop();
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<NegociateWorldModel *>(_world->getRobot(index)->getWorldModel());
            auto *ctl = dynamic_cast<NegociateController *>(_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;
            wm->opp = opp;
            wm->arrival = 1;
            wm->nbOnOpp = opp->getNbNearbyRobots();

            // Add information about his previous investment
            wm->appendOwnInvest(wm->_cooperationLevel);
            if (NegociateSharedData::onlyOtherInTotalInv)
            {
                wm->appendTotalInvest(opp->getCoop());
            }
            else
            {
                wm->appendTotalInvest(opp->getCoop() + wm->_cooperationLevel);
            }
            wm->appendToCommonKnowledgeReputation(wm->_cooperationLevel);

            //Reward him
            ctl->increaseFitness(0);
            if (index == 0)
            {
                opp->curInv = opp->getCoop() + wm->_cooperationLevel;
            }
        }
    }
}

void NegociateAnalysisWorldObserver::resetEnvironment()
{
    /* Put all robots in a hidden place */
    for (int i = 0; i < gNbOfRobots; i++)
    {
        Robot *robot = gWorld->getRobot(i);
        robot->unregisterRobot();
        robot->setCoord(0, 0);
        robot->setCoordReal(0, 0);
    }

    for (auto* obj : gPhysicalObjects)
    {

        /* register objects */
        obj->unregisterObject();
        obj->resetLocation();
        obj->registerObject();

        /* Make them ESS */
        auto* opp = dynamic_cast<NegociateAnalysisOpportunity*>(obj);
        opp->setCoopValue(2.5);
    }

    /* set up the target opportunity */
    auto *opp = dynamic_cast<NegociateAnalysisOpportunity*>(gPhysicalObjects[0]);
    opp->setCoopValue(m_curCoop);
    opp->setNbFakeRobots(m_curnbrob);
    opp->fakerobots.clear();
    for (int i = 0; i < m_curnbrob; i++){
        opp->fakerobots.push_back(gWorld->getRobot(i + gNbOfPhysicalObjects));
    }
    for (auto* obj : gPhysicalObjects)
    {
        auto* opp = dynamic_cast<NegociateAnalysisOpportunity*>(obj);
        opp->placeFakeRobot();
    }

    /* place robot on target */
    Robot *robot = gWorld->getRobot(0);
    robot->unregisterRobot();
    dynamic_cast<NegociateWorldModel *>(robot->getWorldModel())->reset();
    int xrob = gPhysicalObjects[0]->getXCenterPixel() + gPhysicalObjectDefaultRadius + 1;
    int yrob = gPhysicalObjects[0]->getYCenterPixel();
    robot->setCoord(xrob, yrob);
    robot->setCoordReal(xrob, yrob);
    robot->registerRobot();

    //Register our robot on the spot
    robot->callObserverPost();
    registerRobotsOnOpportunities();
    computeOpportunityImpact();
    assert(dynamic_cast<NegociateWorldModel *>(robot->getWorldModel())->opp == gPhysicalObjects[0]);
    assert(dynamic_cast<NegociateAnalysisOpportunity *>(gPhysicalObjects[0])->getNbNearbyRobots() == m_curnbrob + 1);

    if (NegociateSharedData::tpToNewObj)
    {
        robot->getWorldModel()->_agentAbsoluteOrientation = 0;
    }
    dynamic_cast<NegociateWorldModel *>(robot->getWorldModel())->selfA = NegociateSharedData::meanA;



}

void NegociateAnalysisWorldObserver::initObjects() const
{
    /*int i = 0;
    int objPerCoop = (int) ceil((double) gNbOfPhysicalObjects / 12); // coop : 0, 1, .., 10 + obj without ind
    int nbRob = 0;
    int coop = 0;
    for (auto *phys : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<NegociateAnalysisOpportunity *>(phys);
        opp->setCoopValue(coop);
        opp->setNbFakeRobots(nbRob);
        if (nbRob > 0)
        {
            std::shared_ptr<Robot> rob = std::make_shared<Robot>(gWorld);
            rob->getWorldModel()->_cameraSensorsNb = 0;
            gWorld->addRobot(rob.get());
            opp->fakerobot = rob;
        }
        i++;
        if (i % objPerCoop == 0)
        {
            if (nbRob == 0)
            {
                nbRob = 1;
            }
            else
            {
                coop += NegociateSharedData::maxCoop / 10;
            }
        }
        phys->unregisterObject();
        phys->resetLocation();
        phys->registerObject();
        opp->placeFakeRobot();
    }*/

    /* Robots creation */
    for (int i = 0; i < m_maxrobnb + gNbOfPhysicalObjects; i++)
    {
        Robot* rob = new Robot(gWorld);
        rob->getWorldModel()->_cameraSensorsNb = 0; // Dummy robot
        rob->getWorldModel()->setAlive(false);
        gWorld->addRobot(rob);
    }


    /* register objects */
    for (auto obj : gPhysicalObjects)
    {
        obj->unregisterObject();
        obj->resetLocation();
        obj->registerObject();
    }


    /* attach fake robots for outside option objects */
    for (int i = 1; i < gNbOfPhysicalObjects; i++)
    {
        auto *opp = dynamic_cast<NegociateAnalysisOpportunity*>(gPhysicalObjects[i]);
        opp->setNbFakeRobots(1);
        opp->fakerobots.push_back(gWorld->getRobot(i));

    }


}

void NegociateAnalysisWorldObserver::loadGenome(const std::vector<double> &weights)
{
    auto *ctl = dynamic_cast<NegociateController *>(gWorld->getRobot(0)->getController());
    ctl->loadNewGenome(weights);
    ctl->resetFitness();
}



void NegociateAnalysisWorldObserver::addObjectToTeleport(int id)
{
    objectsToTeleport.emplace(id);
}