/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <CoopFixed2/include/CoopFixed2WorldModel.h>
#include <CoopFixed2/include/CoopFixed2WorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"
#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;



CoopFixed2WorldObserver::CoopFixed2WorldObserver(World *__world) : WorldObserver(__world), m_swapfake(false)
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

    CoopFixed2SharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tind\trep\tfitness\n");

    std::vector<std::string> url;
    if (gRemote.empty())
    {
        std::cerr << "[WARNING] gRemote needs to be defined.";
        url.emplace_back("127.0.0.1");
        url.emplace_back("1703");
    }
    else
    {
        boost::split(url, gRemote, boost::is_any_of(":"));
    }
    pyevo.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_nbIndividuals = 50;
    gMaxIt = -1;
}


CoopFixed2WorldObserver::~CoopFixed2WorldObserver()
{
    delete m_fitnessLogManager;
};

void CoopFixed2WorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;
    if (CoopFixed2SharedData::fakeRobots) {
        m_nbFakeRobots = gNbOfRobots / 2;
    }
    else {
        m_nbFakeRobots = 0;
    }

    m_fakerobotslist = std::vector<int>(m_nbIndividuals, false);
    for(int i = 0; i < m_nbFakeRobots; i++)
    {
        m_fakerobotslist[i] = true;
    }
    std::shuffle(m_fakerobotslist.begin(), m_fakerobotslist.end(), engine);
    m_swapfake = true;

    // Init fitness
    clearRobotFitnesses();

    // create individuals
    int nbweights = dynamic_cast<CoopFixed2Controller * >(m_world->getRobot(0)->getController())->getWeights().size();
    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights);
    m_fitnesses.resize(m_nbIndividuals, 0);
    m_curfitnesses.resize(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();
}


void CoopFixed2WorldObserver::stepPre()
{
    m_curEvaluationIteration++;

    if (m_curEvaluationIteration == CoopFixed2SharedData::evaluationTime)
    {
        m_curEvaluationIteration = 0;
        for (int i = 0; i < m_nbIndividuals; i++)
        {
            auto *wm = m_world->getRobot(i)->getWorldModel();
            m_fitnesses[i] += wm->_fitnessValue;
            m_curfitnesses[i] = wm->_fitnessValue;
        }
        logFitnesses(m_fitnesses);
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;
        if (CoopFixed2SharedData::fakeRobots) {
            if (m_swapfake) {
                for (auto &isfake : m_fakerobotslist) {
                    isfake = 1 - isfake;
                }
            } else {
                std::shuffle(m_fakerobotslist.begin(), m_fakerobotslist.end(), engine);
            }
            m_swapfake = !m_swapfake;
        }
        resetEnvironment();
    }
    if( m_curEvaluationInGeneration == CoopFixed2SharedData::nbEvaluationsPerGeneration)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
    }
}

void CoopFixed2WorldObserver::stepPost()
{
    /* Teleport robots */
    if (CoopFixed2SharedData::teleportRobots) {
        for (auto rid: robotsToTeleport) {
            m_world->getRobot(rid)->unregisterRobot();
            m_world->getRobot(rid)->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                       gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
            m_world->getRobot(rid)->registerRobot();
        }
    }
    if (CoopFixed2SharedData::tpToNewObj)
    {
        auto randomPhys = std::uniform_int_distribution<int>(0, gNbOfPhysicalObjects - 1);
        for (int i = 0; i < m_world->getNbOfRobots(); i++)
        {
            auto *rob = m_world->getRobot(i);
            if (rob->getWorldModel()->_desiredTranslationalValue > 0)
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
    robotsToTeleport.clear();
    registerRobotsOnOpportunities();
    computeOpportunityImpacts();
    if ((m_generationCount+1) % CoopFixed2SharedData::logEveryXGen == 0) {
        if (CoopFixed2SharedData::takeVideo) {
            saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
        }
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0) {
            if (m_logall.is_open()) {
                m_logall.close();
            }
            m_logall.open(gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt");
            m_logall << "eval\titer\tid\ta\tfake\tonOpp\tnbOnOpp\tcurCoop\tmeanOwn\tmeanTotal\n";
        }
        for (int i = 0; i < m_world->getNbOfRobots(); i++) {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(i)->getWorldModel());
            double nbOnOpp = wm->nbOnOpp;
            if (CoopFixed2SharedData::fixRobotNb && nbOnOpp > 2)
            {
                nbOnOpp = 2;
            }
            m_logall << m_curEvaluationInGeneration << "\t"
                     << m_curEvaluationIteration << "\t"
                     << i << "\t"
                     << wm->selfA << "\t"
                     << wm->fake << "\t"
                     << wm->onOpportunity << "\t"
                     << nbOnOpp << "\t"
                     << wm->_cooperationLevel * (int) wm->onOpportunity << "\t"
                     << wm->meanLastOwnInvest() << "\t"
                     << wm->meanLastTotalInvest() << "\n";
        }
    }
    else if ((m_generationCount+1) % CoopFixed2SharedData::logEveryXGen == 1 && m_curEvaluationIteration == 0)
    {
        m_logall.flush(); // Let's flush now that we have written everything.
        m_logall.close();
    }

}


void CoopFixed2WorldObserver::stepEvolution()
{
    if ((m_generationCount+1) % CoopFixed2SharedData::logEveryXGen == 0)
    {
        std::string path = gLogDirectoryname + "/genomes_" + std::to_string(m_generationCount) + ".txt";
        std::ofstream genfile(path);
        genfile << json(m_individuals);
    }
    m_individuals = pyevo.getNextGeneration(m_fitnesses);
    if (m_individuals.empty())
    {
        exit(0);
    }
    m_fitnesses = std::vector<double>(m_nbIndividuals, 0);
    loadGenomesInRobots(m_individuals);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> CoopFixed2WorldObserver::getSortedFitnesses() const
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

void CoopFixed2WorldObserver::logFitnesses(const std::vector<double>& curfitness)
{
    std::stringstream out;
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        out << m_generationCount << "\t" << i << "\t" << m_curEvaluationInGeneration << "\t" << curfitness[i] << "\n";
    }
    m_fitnessLogManager->write(out.str());
    m_fitnessLogManager->flush();
}

void CoopFixed2WorldObserver::resetEnvironment()
{
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
        auto* rwm = dynamic_cast<CoopFixed2WorldModel *>(robot->getWorldModel());
        rwm->lastReputation.clear();
    }


    for (auto* object: gPhysicalObjects)
    {
        dynamic_cast<CoopFixed2Opportunity *>(object)->resetLife();
        object->resetLocation();
        object->registerObject();
    }
    int nb_fake_done = 0;
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        auto *wm = dynamic_cast<CoopFixed2WorldModel*>(robot->getWorldModel());
        if (m_fakerobotslist[iRobot])
        {
            wm->fake = true;
            if (nb_fake_done < m_nbFakeRobots / 2){
                wm->fakeCoef = 1.0/CoopFixed2SharedData::fakeCoef;
            } else {
                wm->fakeCoef = CoopFixed2SharedData::fakeCoef;
            }
            nb_fake_done++;
        } else {
            wm->fake = false;
            wm->fakeCoef = 1;
        }
        wm->setNewSelfA();
    }
}

void CoopFixed2WorldObserver::computeOpportunityImpacts()
{
    const double b = CoopFixed2SharedData::b;
    const double d = CoopFixed2SharedData::d;


    // Mark all robots as not on an cooperation opportunity
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *wm = dynamic_cast<CoopFixed2WorldModel*>(m_world->getRobot(i)->getWorldModel());
        wm->onOpportunity = false;
        wm->nbOnOpp = 0;
        wm->arrival = 0;
    }
    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<CoopFixed2Opportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (CoopFixed2SharedData::fixRobotNb && n >2) {
            n = 2;
        }

        int arrival = 1;
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(index)->getWorldModel());
            wm->onOpportunity = !CoopFixed2SharedData::fixRobotNb || arrival <= 2;
            wm->nbOnOpp = opp->getNbNearbyRobots();
            wm->arrival = arrival;
            arrival++;
        }

        // If we only give reward for the first two robots
        if (CoopFixed2SharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
        {
            itmax = opp->getNearbyRobotIndexes().begin() + 2;
        }

        for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
        {
            const auto * const wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(*index)->getWorldModel());
            const double coop = clamp(wm->_cooperationLevel * wm->fakeCoef, 0, CoopFixed2SharedData::maxCoop);
            totalInvest += coop;
            totalA += wm->selfA;
        }

        for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(*index)->getWorldModel());
            const double coop = clamp(wm->_cooperationLevel * wm->fakeCoef, 0, CoopFixed2SharedData::maxCoop);
            wm->appendOwnInvest(coop);
            if (n >= 2)
            {
                wm->appendToReputation(coop);
            }
            if (CoopFixed2SharedData::onlyOtherInTotalInv)
            {
                wm->appendTotalInvest(totalInvest - coop);
            } else {
                wm->appendTotalInvest(totalInvest);
            }

            if (!wm->fake) {
                wm->_fitnessValue += payoff(coop, totalInvest, n, wm->selfA, b, d);
            }
        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;

    }
}

double CoopFixed2WorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a, const double b, const double d)
{
    double res = 0;
    const double x0 = (totalInvest - invest);
    if (!CoopFixed2SharedData::prisonerDilemma)
    {
        if (CoopFixed2SharedData::atLeastTwo and n < 2) {
            res = 1;
        } else {
            res = (a * totalInvest + b * x0) / n - 0.5 * invest * invest + d * x0 * invest / n;
        }
    }
    else
    {
        throw std::runtime_error("Not implemented");
    }
    return res;
}


void CoopFixed2WorldObserver::registerRobotsOnOpportunities()
{
    for (auto *physicalObject : gPhysicalObjects)
    {
        auto *opp = dynamic_cast<CoopFixed2Opportunity *>(physicalObject);
        opp->registerNewRobots();
    }
}

void CoopFixed2WorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<CoopFixed2Controller*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

void CoopFixed2WorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>>& genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto* ctl = dynamic_cast<CoopFixed2Controller *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}

void CoopFixed2WorldObserver::addRobotToTeleport(int robotId)
{

    robotsToTeleport.emplace(robotId);
}

void CoopFixed2WorldObserver::addObjectToTeleport(int id) {
    objectsToTeleport.emplace(id);
}
