//
// Created by paul on 30/10/17.
//


#include <WorldModels/RobotWorldModel.h>
#include <FastWanderer/include/FastWandererWorldObserver.h>
#include "RoboroboMain/main.h"
#include "FastWanderer/include/FastWandererSharedData.h"
#include <boost/algorithm/string.hpp>

FastWandererWorldObserver::FastWandererWorldObserver(World *__world) : WorldObserver(__world)
{
    std::vector<std::string> url;
    if (gRemote == "")
    {
        std::cerr << "[WARNING] gRemote needs to be defined.";
        url.push_back("127.0.0.1");
        url.push_back("1703");
    }
    boost::split(url, gRemote, boost::is_any_of(":"));
    pycma.connect(url[0], static_cast<unsigned short>(std::stol(url[1])));

    m_curEvalutionIteration = 0;
    _generationCount = 0;

    FastWandererSharedData::initSharedData();

    // Log files

    std::string fitnessLogFilename = gLogDirectoryname + "/fitnesslog.txt";
    m_fitnessLogManager = new LogManager(fitnessLogFilename);
    m_fitnessLogManager->write("gen\tpop\tminfit\tq1fit\tmedfit\tq3fit\tmaxfit\tmeanfit\tvarfit\n");

    gMaxIt = -1;
}


FastWandererWorldObserver::~FastWandererWorldObserver()
{
    delete m_fitnessLogManager;
}

void FastWandererWorldObserver::reset()
{
    int nbweights = dynamic_cast<FastWandererController *>(gWorld->getRobot(0)->getController())->getGenome().size();
    loadNextGeneration(pycma.initCMA(gInitialNumberOfRobots, nbweights));
    resetEnvironment();
    clearRobotFitnesses();
}

void FastWandererWorldObserver::stepPre()
{
    if (m_curEvalutionIteration  == FastWandererSharedData::evaluationTime)
    {
        logFitnesses(getSortedFitnesses());
        std::vector<double> fitness;
        for (int i = 0; i < gWorld->getNbOfRobots(); i++)
        {
            fitness.push_back(gWorld->getRobot(i)->getWorldModel()->_fitnessValue);
        }
        std::vector<std::vector<double>> new_generation = pycma.getNextGeneration(fitness);
        if (new_generation.empty())
        {
            exit(0);
        }
        else
        {
            loadNextGeneration(new_generation);
        }
        resetEnvironment();
        clearRobotFitnesses();
        m_curEvalutionIteration = 0;
    }
    m_curEvalutionIteration++;
}

void FastWandererWorldObserver::loadNextGeneration(const std::vector<std::vector<double>> &generation)
{
    for (int i = 0; i < gWorld->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<FastWandererController *>(gWorld->getRobot(i)->getController());
        ctl->loadNewGenome(generation[i]);
    }
}

void FastWandererWorldObserver::stepPost()
{
    for (int i = 0; i < gWorld->getNbOfRobots(); i++)
    {
        RobotWorldModel *wm = gWorld->getRobot(i)->getWorldModel();
        double speed = (wm->_actualTranslationalValue / gMaxTranslationalSpeed);
        double rotspeed = (wm->_actualRotationalVelocity / gMaxRotationalSpeed);
        double closestObjDistance = 1;
        for (int j = 0; j < wm->_cameraSensorsNb; j++)
        {
            double cur_dist = wm->getDistanceValueFromCameraSensor(j) / wm->getCameraSensorMaximumDistanceValue(j);
            if (cur_dist < closestObjDistance)
            {
                closestObjDistance = cur_dist;
            }
        }
        wm->_fitnessValue += speed * (1 - sqrt(fabs(rotspeed))) * closestObjDistance;
    }
}


std::vector<std::pair<int, double>> FastWandererWorldObserver::getSortedFitnesses() const
{
    std::vector<std::pair<int, double>> fitnesses(gWorld->getNbOfRobots());
    for (int i = 0; i < gWorld->getNbOfRobots(); i++)
    {
        fitnesses[i].first = i;
        fitnesses[i].second = gWorld->getRobot(i)->getWorldModel()->_fitnessValue;
    }
    std::sort(fitnesses.begin(), fitnesses.end(),
              [](std::pair<int, double>a, std::pair<int, double>b){return a.second < b.second;});
    return fitnesses;
}

void FastWandererWorldObserver::logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses)
{
    unsigned long size = sortedFitnesses.size();

    double sum = std::accumulate(sortedFitnesses.begin(), sortedFitnesses.end(), 0.0,
                                 [](double a, const std::pair<int, double>& b) -> double {return a + b.second;});
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
    m_fitnessLogManager->write(out.str());
}

void FastWandererWorldObserver::resetEnvironment()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }

    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        robot->registerRobot();
    }
}


void FastWandererWorldObserver::clearRobotFitnesses()
{
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        auto* ctl = dynamic_cast<FastWandererController*>(gWorld->getRobot(iRobot)->getController());
        ctl->resetFitness();
    }
}