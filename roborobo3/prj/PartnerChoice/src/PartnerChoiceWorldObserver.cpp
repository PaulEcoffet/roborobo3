//
// Created by paul on 30/10/17.
//

#include <PartnerChoice/include/PartnerChoiceWorldModel.h>
#include <core/Utilities/Graphics.h>
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

    initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/observer.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");
    std::cout << "gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n";
    std::string genomeLogFilename = gLogDirectoryname + "/genome.txt";
    m_genomeLogManager = new LogManager(genomeLogFilename);
}

PartnerChoiceWorldObserver::~PartnerChoiceWorldObserver()
{
    delete m_fitnessLogManager;
    delete m_genomeLogManager;
};

void PartnerChoiceWorldObserver::initOpportunities()
{
    double curCoopVal = 0;
    double stepCoop = PartnerChoiceSharedData::maxCoop / ((double)PartnerChoiceSharedData::nbCoopStep - 1);
    int stepEvery = gNbOfPhysicalObjects / PartnerChoiceSharedData::nbCoopStep;
    int i = 0;
    auto physObjList = gPhysicalObjects;
    std::mt19937 mt(time(0)); // TODO: Ugly
    std::shuffle(physObjList.begin(), physObjList.end(), mt);
    for (auto *physicalObject : physObjList)
    {
        auto *opp = dynamic_cast<PartnerChoiceOpportunity *>(physicalObject);
        opp->setCoopValue(curCoopVal);
        if (i == stepEvery)
        {
            curCoopVal += stepCoop;
            i = 0;
        }
        i++;
    }
}

void PartnerChoiceWorldObserver::step()
{
    if ((_generationCount + 1) % PartnerChoiceSharedData::takeVideoEveryGeneration == 0)
    {
        std::string name = "gen_" + std::to_string(_generationCount);
        saveCustomScreenshot(name);
    }
    computeOpportunityImpact();
    clearOpportunityNearbyRobots();

    if (m_curEvalutionIteration == PartnerChoiceSharedData::evaluationTime)
    {
        resetEnvironment();
        m_curEvalutionIteration = 0;
        m_curEvaluationInGeneration++;
    }
    if( m_curEvaluationInGeneration == PartnerChoiceSharedData::nbEvaluationsPerGeneration )
    {
        stepEvaluation();
        _generationCount++;
        m_curEvaluationInGeneration = 0;
        m_curEvalutionIteration = 0;
    }
    else
    {
        m_curEvalutionIteration++;
    }
    if (m_curEvalutionIteration  == PartnerChoiceSharedData::evaluationTime / 2)
    {
        initOpportunities();
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
    std::vector<std::pair<int, double>> fitnesses(static_cast<unsigned long>(m_world->getNbOfRobots()));
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<PartnerChoiceController *>(m_world->getRobot(i)->getController());
        fitnesses[i].first = i;
        fitnesses[i].second = ctl->getFitness();
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
    m_fitnessLogManager->flush();
}

void PartnerChoiceWorldObserver::logGenomes(const std::vector<std::pair<int, double>>& sortedFitnesses)
{
    std::stringstream outhead;
    m_genomeLogManager->write("BEGINGROUP\n");
    outhead << _generationCount << "\n";
    m_genomeLogManager->write(outhead.str());
    int curRank = static_cast<int>(sortedFitnesses.size());
    for (const auto &elem : sortedFitnesses)
    {
        std::stringstream out;
        auto* ctl = dynamic_cast<PartnerChoiceController *>(m_world->getRobot(elem.first)->getController());
        PartnerChoiceController::genome genome = ctl->getGenome();
        out << "BEGININD\n";
        out << "ID=" << elem.first << "\n";
        out << "RANK=" << curRank << "\n";
        out << "BEGINGEN\n";
        out << genome.sigma << "\n";
        out << genome.weights.size() << "\n";
        for (const auto& weight : genome.weights)
        {
            out << weight << " ";
        }
        out << "ENDGEN" << "\n";
        out << "ENDIND" << "\n";
        m_genomeLogManager->write(out.str());
        curRank--;
    }
    m_genomeLogManager->write("ENDGROUP\n");
    m_genomeLogManager->flush();
}

void PartnerChoiceWorldObserver::createNextGeneration(const std::vector<std::pair<int, double>>& fitnesses)
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::vector<double> fitWeights;
    std::transform(fitnesses.begin(), fitnesses.end(), std::back_inserter(fitWeights),
                   [](const std::pair<int, double>& a){return a.second;});
    std::discrete_distribution<int> drawParent(fitWeights.begin(), fitWeights.end());
    std::vector<PartnerChoiceController::genome> newGenomes;
    std::vector<int> nbPicked(fitWeights.size(), 0);
    newGenomes.reserve(static_cast<unsigned long>(m_world->getNbOfRobots()));

    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        int parentFitId = drawParent(mt);
        int parentId = fitnesses[parentFitId].first;
        auto *parentCtl = dynamic_cast<PartnerChoiceController *>(m_world->getRobot(parentId)->getController());
        newGenomes.push_back(parentCtl->getGenome());
        nbPicked[parentFitId]++;
    }
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *childCtl = dynamic_cast<PartnerChoiceController *>(m_world->getRobot(i)->getController());
        childCtl->loadNewGenome(newGenomes[i]);
        childCtl->mutateGenome();
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

void PartnerChoiceWorldObserver::initSharedData()
{
    gProperties.checkAndGetPropertyValue("evaluationTime", &PartnerChoiceSharedData::evaluationTime, true);
    gProperties.checkAndGetPropertyValue("genomeLog", &PartnerChoiceSharedData::genomeLog, true);
    gProperties.checkAndGetPropertyValue("sigma", &PartnerChoiceSharedData::sigma, true);
    gProperties.checkAndGetPropertyValue("controllerType", &PartnerChoiceSharedData::controllerType, true);
    gProperties.checkAndGetPropertyValue("gMaxTranslationalSpeed", &PartnerChoiceSharedData::maxTranslationalValue, true);
    gProperties.checkAndGetPropertyValue("gMaxRotationalSpeed", &PartnerChoiceSharedData::maxRotationalVelocity, true);
    gProperties.checkAndGetPropertyValue("nbHiddenLayers", &PartnerChoiceSharedData::nbHiddenLayers, true);
    gProperties.checkAndGetPropertyValue("nbNeuronsPerHiddenLayer", &PartnerChoiceSharedData::nbNeuronsPerHiddenLayer, true);
    gProperties.checkAndGetPropertyValue("maxCoop", &PartnerChoiceSharedData::maxCoop, true);
    gProperties.checkAndGetPropertyValue("nbCoopStep", &PartnerChoiceSharedData::nbCoopStep, true);
    gProperties.checkAndGetPropertyValue("constantA", &PartnerChoiceSharedData::constantA, true);
    gProperties.checkAndGetPropertyValue("constantK", &PartnerChoiceSharedData::constantK, true);
    gProperties.checkAndGetPropertyValue("nbEvaluationsPerGeneration", &PartnerChoiceSharedData::nbEvaluationsPerGeneration, true);
    gProperties.checkAndGetPropertyValue("takeVideoEveryGeneration", &PartnerChoiceSharedData::takeVideoEveryGeneration, false);
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
}

void PartnerChoiceWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<PartnerChoiceController*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}

