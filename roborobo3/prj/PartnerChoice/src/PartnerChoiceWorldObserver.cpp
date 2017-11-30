//
// Created by paul on 30/10/17.
//

#include <PartnerChoice/include/PartnerChoiceWorldModel.h>
#include <core/Utilities/Graphics.h>
#include <PartnerChoice/include/PartnerChoiceAnalysisWorldObserver.h>
#include <PartnerChoice/include/PartnerChoiceWorldObserver.h>
#include "core/RoboroboMain/main.h"
#include "PartnerChoice/include/PartnerChoiceWorldObserver.h"
#include "PartnerChoice/include/PartnerChoiceController.h"
#include "PartnerChoice/include/PartnerChoiceSharedData.h"


PartnerChoiceWorldObserver::PartnerChoiceWorldObserver(World *__world) : WorldObserver(__world)
{
    m_world = __world;

    m_curEvalutionIteration = 0;
    m_curEvaluationInGeneration = 0;
    _generationCount = 0;
    m_curInd = 0;

    std::random_device rd;
    m_mt = std::mt19937(rd());

    PartnerChoiceSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");
    std::cout << "gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n";

    m_observer = new LogManager(gLogDirectoryname + "/observer.txt");
    m_observer->write("gen\tind\teval\titer\tonOpp\tcoop\n");

    m_nbIndividuals = 50;
    gMaxIt = (long long) m_nbIndividuals * (long long)PartnerChoiceSharedData::evaluationTime * (long long)PartnerChoiceSharedData::nbEvaluationsPerGeneration * 2000;
}


PartnerChoiceWorldObserver::~PartnerChoiceWorldObserver()
{
    delete m_fitnessLogManager;
    delete m_observer;
};

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
    std::shuffle(physObjList.begin(), physObjList.end(), m_mt);
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
    monitorPopulation();
    computeOpportunityImpact();
    clearOpportunityNearbyRobots();

    m_curEvalutionIteration++;

    if (m_curEvalutionIteration == PartnerChoiceSharedData::evaluationTime)
    {
        resetEnvironment();
        m_curEvalutionIteration = 0;
        m_curEvaluationInGeneration++;
    }
    if( m_curEvaluationInGeneration == PartnerChoiceSharedData::nbEvaluationsPerGeneration)
    {
        m_individuals[m_curInd].fitness = m_world->getRobot(0)->getWorldModel()->_fitnessValue;
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
        stepEvaluation();
        m_curInd = 0;
        activateOnlyRobot(m_curInd);
        _generationCount++;
    }
}

void PartnerChoiceWorldObserver::monitorPopulation() const
{
    if ((_generationCount + 1) % PartnerChoiceSharedData::takeVideoEveryGeneration == 0 && m_curInd % 25 == 0)
    {
        std::string name = "gen_" + std::to_string(_generationCount) + "_ind_" + std::to_string(m_curInd);
        saveCustomScreenshot(name);
    }

    if ((_generationCount + 1) % PartnerChoiceSharedData::takeVideoEveryGeneration == 0)
    {
        std::stringstream out;
        auto *wm = dynamic_cast<PartnerChoiceWorldModel *>(m_world->getRobot(0)->getWorldModel());
        out << _generationCount << "\t";
        out << m_curInd << "\t";
        out << m_curEvaluationInGeneration << "\t";
        out << m_curEvalutionIteration << "\t";
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

void PartnerChoiceWorldObserver::stepEvaluation()
{
    std::vector<std::pair<int, double>> fitnesses = getSortedFitnesses();
    logFitnesses(fitnesses);
    if ((_generationCount + 1) % PartnerChoiceSharedData::genomeLog == 0)
    {
        logGenomes(fitnesses);
    }
    createNextGeneration(fitnesses);
    clearRobotFitnesses();
}

std::vector<std::pair<int, double>> PartnerChoiceWorldObserver::getSortedFitnesses() const
{
    std::vector<std::pair<int, double>> fitnesses(m_individuals.size());
    for (int i = 0; i < m_individuals.size(); i++)
    {
        const auto &individual = m_individuals[i];
        fitnesses[i].first = i;
        fitnesses[i].second = individual.fitness;
    }
    std::sort(fitnesses.begin(), fitnesses.end(),
              [](std::pair<int, double>a, std::pair<int, double>b){return a.second < b.second;});
    return fitnesses;
}

void PartnerChoiceWorldObserver::logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses)
{
    unsigned long size = sortedFitnesses.size();

    double sum = std::accumulate(sortedFitnesses.begin(), sortedFitnesses.end(), 0.0,
                                 [](double& a, const std::pair<int, double>& b) -> double {return a + b.second;});
    double mean = sum / size;

    std::vector<double> diff(size);
    std::transform(sortedFitnesses.begin(), sortedFitnesses.end(), diff.begin(),
                   [mean](std::pair<int, double> x) { return x.second - mean; });
    double variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / size;


    std::stringstream out;
    out << _generationCount << "\t";
    out << size << "\t";
    out << sortedFitnesses[0].second << "\t"; // MIN
    out << sortedFitnesses[size/4].second << "\t"; // 1st quartile
    out << sortedFitnesses[size/2].second << "\t"; // 2nd quartile - Median
    out << sortedFitnesses[(3*size)/4].second << "\t"; // 3rd quartile
    out << sortedFitnesses[size-1].second << "\t"; // Max
    out << mean << "\t";
    out << variance << "\n";
    std::cout << out.str();
    m_fitnessLogManager->write(out.str());
}

void PartnerChoiceWorldObserver::logGenomes(const std::vector<std::pair<int, double>>& sortedFitnesses)
{
    int curRank = static_cast<int>(sortedFitnesses.size());
    for (const auto &elem : sortedFitnesses)
    {
        json curInd;

        PartnerChoiceController::genome genome = m_individuals[elem.first].genome;
        curInd["id"] = elem.first;
        curInd["generation"] = _generationCount;
        curInd["rank"] = curRank;
        curInd["sigma"] = genome.sigma;
        curInd["weights"] = genome.weights;
        m_genomesLogJson.push_back(curInd);
        curRank--;
    }
    std::ofstream genomeLogFile(gLogDirectoryname + "/genome.txt");
    genomeLogFile << std::setw(4) << m_genomesLogJson << std::endl;
}

void PartnerChoiceWorldObserver::createNextGeneration(const std::vector<std::pair<int, double>>& fitnesses)
{

    std::vector<double> fitWeights;
    std::transform(fitnesses.begin(), fitnesses.end(), std::back_inserter(fitWeights),
                   [fitnesses](const std::pair<int, double>& a){return a.second;});
    //std::discrete_distribution<int> drawParent(fitWeights.begin(), fitWeights.end());
    std::uniform_int_distribution<int> drawParent(40, 49);
    std::vector<PartnerChoiceController::genome> newGenomes;
    std::vector<int> nbDrawn(m_individuals.size(), 0);
    newGenomes.reserve(m_individuals.size());

    for (int i = 0; i < m_individuals.size() - 10; i++)
    {
        int parentFitId = drawParent(m_mt);
        int parentId = fitnesses[parentFitId].first;
        newGenomes.push_back(m_individuals[parentId].genome);
        nbDrawn[parentFitId]++;
    }
    for (int i = m_individuals.size() - 10; i < m_individuals.size(); i++)
    {
        int parentId = fitnesses[i].first;
        newGenomes.push_back(m_individuals[parentId].genome);
    }
    for (int i = 0; i < m_individuals.size(); i++)
    {
        if (i < m_individuals.size() - 10)
        {
            m_individuals[i].genome = newGenomes[i].mutate();
        }
        else
        {
            m_individuals[i].genome = newGenomes[i];
        }
    }

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
            auto *ctl = dynamic_cast<PartnerChoiceController*>(m_world->getRobot(index)->getController());

            // Mark the robot on an opportunity
            wm->onOpportunity = true;

            // Add information about his previous investment
            wm->appendOwnInvest(0);
            wm->appendTotalInvest(opp->getCoop());

            //Reward him
            ctl->increaseFitness(opp->getCoop());
        }
    }
}

double PartnerChoiceWorldObserver::payoff(const double invest, const double totalInvest) const
{
    const int nbRobots = 2;
    const double coeff = PartnerChoiceSharedData::constantK / (1.0 + pow(nbRobots - 2, 2)); // \frac{k}{1+(n-2)^2}
    const double res = coeff * pow(totalInvest, PartnerChoiceSharedData::constantA) - invest;
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

void PartnerChoiceWorldObserver::reset()
{
    WorldObserver::reset();

    // Init environment

    initOpportunities();

    // Init fitness

    clearRobotFitnesses();

    // create individuals

    m_individuals.reserve(m_nbIndividuals);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        individual ind{};
        ind.fitness = 0;
        PartnerChoiceController::genome gen;
        gen.sigma = PartnerChoiceSharedData::sigma;
        gen.weights.resize(dynamic_cast<PartnerChoiceController *>(m_world->getRobot(0)->getController())->getGenome().weights.size());
        for (auto& weight: gen.weights)
        {
            weight = random() * 2 - 1;
        }
        ind.genome = gen;
        m_individuals.push_back(ind);
    }

    activateOnlyRobot(m_curInd);
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
    ctl->loadNewGenome(m_individuals[robotIndex].genome);
}

