/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <MaxOne/include/MaxOneWorldModel.h>
#include <MaxOne/include/MaxOneWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "MaxOne/include/MaxOneWorldObserver.h"
#include "MaxOne/include/MaxOneController.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
#include <MaxOne/include/MaxOneAgentObserver.h>
//#include <cv.hpp>

using boost::algorithm::clamp;


MaxOneWorldObserver::MaxOneWorldObserver(World *__world) :
        WorldObserver(__world)
{
    m_world = __world;
    m_curEvaluationIteration = 0;
    m_curEvaluationInGeneration = 0;
    m_generationCount = 0;

}


MaxOneWorldObserver::~MaxOneWorldObserver()
{
}

void MaxOneWorldObserver::reset()
{
    m_nbIndividuals = gNbOfRobots;
    // Init fitness
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

    m_nbIndividuals = gInitialNumberOfRobots;
    gMaxIt = -1;

    double mutproba = 1;
    double mutrate = 0.1;
    gProperties.checkAndGetPropertyValue("mutProb", &mutproba, false);
    gProperties.checkAndGetPropertyValue("mutRate", &mutrate, false);
    // create individuals
    int nbweights = 10;
    std::vector<double> minbounds(nbweights, 0);
    std::vector<double> maxbounds(nbweights, 1);
    std::vector<double> std(nbweights, mutrate);
    std::vector<double> minguess(nbweights, 0);
    std::vector<double> maxguess(nbweights, 1);
    std::vector<double> mutprob(nbweights, mutproba);
    std::vector<bool> randomguess(nbweights, true);

    int train = 0;
    json extra = {{"noextra", "noextra"}};
    gProperties.checkAndGetPropertyValue("train", &train, false);
    if (train == 2)
    {
        int split = 5;
        for (unsigned long i = 0; i < split; i++)
        {
            mutprob[i] = 0;
            randomguess[i] = false;
        }

        extra = {{"randomguess", randomguess}};
    }


    m_individuals = pyevo.initCMA(m_nbIndividuals, nbweights, minbounds, maxbounds, minguess, maxguess, std, mutprob,
            extra);
    loadGenomesInRobots(m_individuals);

    resetEnvironment();
}


void MaxOneWorldObserver::stepPre()
{
    bool resetEnv = false;
    if (m_curEvaluationIteration == 1 )
    {
        m_curEvaluationIteration = 0;
        m_curEvaluationInGeneration++;
        resetEnv = true;
    }
    if (m_curEvaluationInGeneration == 1)
    {
        m_curEvaluationInGeneration = 0;
        stepEvolution();
        m_generationCount++;
        if (resetEnv)
        {
            resetEnvironment();
        }
    }
}

void MaxOneWorldObserver::stepPost()
{
    /* Plays */
    m_curEvaluationIteration++;
}


void MaxOneWorldObserver::stepEvolution()
{
    std::vector<double> fitnesses(m_nbIndividuals);
    for (int i = 0; i < m_nbIndividuals; i++)
    {
        auto ctl = dynamic_cast<MaxOneController*>(m_world->getRobot(i)->getController());
        auto weights = ctl->getWeights();
        double tot = 0;
        for (auto w : weights)
        {
            tot += w;
        }
        if (random01() < 0.1)
        {
            tot += randgaussian();
        }
        fitnesses[i] = tot;
    }
    m_individuals = pyevo.getNextGeneration(m_individuals, fitnesses);
    if (m_individuals.empty())
    {
        exit(0);
    }
    loadGenomesInRobots(m_individuals);
}


void MaxOneWorldObserver::resetEnvironment()
{
}


void MaxOneWorldObserver::loadGenomesInRobots(const std::vector<std::vector<double>> &genomes)
{
    assert(genomes.size() == m_nbIndividuals);
    for (int i = 0; i < m_world->getNbOfRobots(); i++)
    {
        auto *ctl = dynamic_cast<MaxOneController *>(m_world->getRobot(i)->getController());
        ctl->loadNewGenome(genomes[i]);
    }

}