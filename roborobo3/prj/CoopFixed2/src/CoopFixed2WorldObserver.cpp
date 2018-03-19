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


CoopFixed2WorldObserver::CoopFixed2WorldObserver(World *__world) : WorldObserver(__world)
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
    m_nbFakeRobots = gNbOfRobots - m_nbIndividuals; // Useless so far
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
        logFitnesses(m_curfitnesses);
        clearRobotFitnesses();
        m_curEvaluationInGeneration++;

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
    if ((m_generationCount+1) % 1000 == 0) {
        if (CoopFixed2SharedData::takeVideo && gMovie) {
            saveCustomScreenshot("movie_gen_" + std::to_string(m_generationCount));
        }
        if (m_curEvaluationIteration == 0 && m_curEvaluationInGeneration == 0) {
            if (m_logall.is_open()) {
                m_logall.close();
            }
            m_logall.open(gLogDirectoryname + "/logall_" + std::to_string(m_generationCount) + ".txt");
            m_logall << "eval\titer\tid\ta\tonOpp\tnbOnOpp\tcurCoop\tmeanOwn\tmeanTotal\n";
        }
        for (int i = 0; i < m_world->getNbOfRobots(); i++) {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(i)->getWorldModel());
            m_logall << m_curEvaluationInGeneration << "\t"
                     << m_curEvaluationIteration << "\t"
                     << i << "\t"
                     << wm->selfA << "\t"
                     << wm->onOpportunity << "\t"
                     << wm->nbOnOpp << "\t"
                     << wm->_cooperationLevel * (int) wm->onOpportunity << "\t"
                     << wm->meanLastOwnInvest() << "\t"
                     << wm->meanLastTotalInvest() << "\n";
        }
    }
    else if ((m_generationCount+1) % 1000 == 1 && m_curEvaluationIteration == 0)
    {
        m_logall.flush(); // Let's flush now that we have written everything.
        m_logall.close();
    }

}


void CoopFixed2WorldObserver::stepEvolution()
{
    if ((m_generationCount+1) % 1000 == 0)
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
    }


    for (auto* object: gPhysicalObjects)
    {
        dynamic_cast<CoopFixed2Opportunity *>(object)->resetLife();
        object->resetLocation();
        object->registerObject();
    }
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        auto *wm = dynamic_cast<CoopFixed2WorldModel* >(robot->getWorldModel());
        wm->setNewSelfA();
    }
}

void CoopFixed2WorldObserver::computeOpportunityImpacts()
{
    const double b = CoopFixed2SharedData::b;


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
        auto *opp = dynamic_cast<CoopFixed2Opportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int arrival = 1;
        for (auto index : opp->getNearbyRobotIndexes())
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(index)->getWorldModel());
            wm->onOpportunity = true;
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
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(*index)->getWorldModel());
            totalInvest += wm->_cooperationLevel;
        }

        for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
        {
            auto *wm = dynamic_cast<CoopFixed2WorldModel *>(m_world->getRobot(*index)->getWorldModel());
            wm->appendOwnInvest(wm->_cooperationLevel);
            wm->appendTotalInvest(totalInvest);
            wm->_fitnessValue += payoff(wm->_cooperationLevel, totalInvest, opp->getNbNearbyRobots(), wm->selfA, b);

        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;

    }
}

double CoopFixed2WorldObserver::payoff(const double invest, const double totalInvest, const int n, const double a, const double b) const
{
    double res = 0;
    if (!CoopFixed2SharedData::prisonerDilemma)
    {

        res = (a * totalInvest + b * (totalInvest - invest)) / n - 0.5 * invest * invest;
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
        ctl->loadNewGenome(genomes[i % m_nbIndividuals]); // cycle through genome, the remaining ones are the fake robots
    }

}

void CoopFixed2WorldObserver::addRobotToTeleport(int robotId)
{

    robotsToTeleport.emplace(robotId);
}

void CoopFixed2WorldObserver::addObjectToTeleport(int id) {
    objectsToTeleport.emplace(id);
}
