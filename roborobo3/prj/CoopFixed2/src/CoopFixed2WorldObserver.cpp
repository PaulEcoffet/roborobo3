/**
 * @author Théotime Grohens ; Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "World/World.h"
#include "Utilities/Misc.h"

#include <cfloat>
#include <random>
#include <CoopFixed2/include/CoopFixed2OpportunityObj.h>
#include "World/MovingObject.h"

CoopFixed2WorldObserver::CoopFixed2WorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&CoopFixed2SharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&CoopFixed2SharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&CoopFixed2SharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&CoopFixed2SharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&CoopFixed2SharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&CoopFixed2SharedData::gEvaluationTime,true);

    gProperties.checkAndGetPropertyValue("gMonitorPositions",&CoopFixed2SharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&CoopFixed2SharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&CoopFixed2SharedData::gNbNeuronsPerHiddenLayer,true);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&CoopFixed2SharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&CoopFixed2SharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&CoopFixed2SharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&CoopFixed2SharedData::gSigma,false);
    
    gProperties.checkAndGetPropertyValue("gTotalEffort", &CoopFixed2SharedData::gTotalEffort, false);
    
    gProperties.checkAndGetPropertyValue("gFakeCoopValue", &CoopFixed2SharedData::gFakeCoopValue, false);
    gProperties.checkAndGetPropertyValue("gNbFakeRobots", &CoopFixed2SharedData::gNbFakeRobots, false);
    
    gProperties.checkAndGetPropertyValue("gFixedEffort", &CoopFixed2SharedData::gFixedEffort, false);
    gProperties.checkAndGetPropertyValue("gFixedEffortValue", &CoopFixed2SharedData::gFixedEffortValue, false);
    
    gProperties.checkAndGetPropertyValue("gGenerationLog", &CoopFixed2SharedData::gGenerationLog, false);
    gProperties.checkAndGetPropertyValue("gTakeVideo", &CoopFixed2SharedData::gTakeVideo, false);

    gProperties.checkAndGetPropertyValue("gConstantA", &CoopFixed2SharedData::gConstantA, true);
    gProperties.checkAndGetPropertyValue("gConstantK", &CoopFixed2SharedData::gConstantK, true);

    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _generationItCount = 0;
    _generationCount = 0;
    
    // * Logfiles

    std::string fitnessLogFilename = gLogDirectoryname + "/observer.txt";
    _fitnessLogManager = new LogManager(fitnessLogFilename);
    _fitnessLogManager->write("GEN\tPOP\tMINFIT\tMAXFIT\tAVGFIT\tQ1FIT\tQ2FIT\tQ3FIT\tSTDDEV\n");
    
    std::string genomeLogFilename = gLogDirectoryname + "/genome.txt";
    _genomeLogManager = new LogManager(genomeLogFilename);
}

CoopFixed2WorldObserver::~CoopFixed2WorldObserver()
{
    delete _fitnessLogManager;
    delete _genomeLogManager;
}

void CoopFixed2WorldObserver::reset()
{
    
}

// Reset the environment, and perform an evolution step
void CoopFixed2WorldObserver::stepEvaluation()
{
    // Save fitness values and genomes before the reset
    double totalFitness = 0;
    int nbTrueRobots = gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots;
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<CoopFixed2Controller::genome> genomes(nbTrueRobots);
    std::vector<int> newGenomePick(gNbOfRobots);
    // don't consider the last gNbFakeRobots robots (they have fixed cooperation levels)
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        CoopFixed2Controller *ctl = dynamic_cast<CoopFixed2Controller *>(robot->getController());
        fitnesses[iRobot] = ctl->getFitness();
        totalFitness += ctl->getFitness();
        genomes[iRobot] = ctl->getGenome();
    }
    
    // Environment stuff
    // unregister everyone
    for (auto object: gPhysicalObjects) {
        object->unregisterObject();
    }
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->unregisterRobot();
    }
    // register objects first because they might have fixed locations, whereas robots move anyway
    for (auto object: gPhysicalObjects)
    {
        object->resetLocation();
        object->registerObject();
    }
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
    }
    
    
    // update genomes (we do it here because Robot::reset() also resets genomes)
    // Create a new generation via selection/mutation
    
    // O(1) fitness-proportionate selection
    // Give everyone a new genome, including the fixed-coop robots
    std::vector<int> nbPicks(gNbOfRobots);
    for (int i = 0; i < gNbOfRobots; i++)
        nbPicks[i] = 0;
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        bool done = false;
        int pick = -1;
        while (done == false) {
            pick = rand()%gNbOfRobots;
            double draw = ranf()*totalFitness;
            if (draw <= fitnesses[pick] && pick < nbTrueRobots) // choose this robot
            {
                done = true;
                nbPicks[pick]++;
            }
        }
        newGenomePick[iRobot] = pick;
    }
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
//        printf("New genome of robot %d: %d\n", iRobot, newGenomePick[iRobot]);
        Robot *robot = gWorld->getRobot(iRobot);
        CoopFixed2Controller *ctl = dynamic_cast<CoopFixed2Controller *>(robot->getController());
        ctl->loadNewGenome(genomes[newGenomePick[iRobot]]);
    }
//    for (int iRobot = 0; iRobot < 40; iRobot++)
//        printf("Robot %.2d was picked %.2lf%% of the time and had proba %.2lf%%\n", iRobot, (double)nbPicks[iRobot]/50.0*100.0, fitnesses[iRobot]/totalFitness*100.0);
}

void CoopFixed2WorldObserver::step()
{
    teleportRobots(_robotsToTeleport);
    _robotsToTeleport.clear();

    computeOpportunityImpact();

    if( _generationItCount == CoopFixed2SharedData::gEvaluationTime - 1 )
    {
        monitorPopulation();
        stepEvaluation();
        _generationCount++;
        _generationItCount = 0;
    }
    else
    {
        _generationItCount++;
    }
    
    updateMonitoring();
    updateEnvironment();
}

void CoopFixed2WorldObserver::computeOpportunityImpact() const
{
    for (auto physicalObj : gPhysicalObjects)
    {
        auto opportunity = dynamic_cast<CoopFixed2OpportunityObj *>(physicalObj);
        if (opportunity->getNearbyRobots().size() == 2)
        {
            opportunity->decrementLockRemainingTime();

            double totalInvest = 0;
            for (auto robotIndex : opportunity->getNearbyRobots())
            {
                auto ctl = dynamic_cast<CoopFixed2Controller *>(gRobots[robotIndex]->getController());
                totalInvest += ctl->getCooperationLevel();
            }
            for (auto robotIndex : opportunity->getNearbyRobots())
            {
                auto ctl = dynamic_cast<CoopFixed2Controller *>(gRobots[robotIndex]->getController());
                ctl->wasNearObject(totalInvest, ctl->getCooperationLevel(),
                                   static_cast<int>(opportunity->getNearbyRobots().size()));
            }
        }
        else
        {
            opportunity->setLockRemainingTime(0);
        }
        bool canMove = opportunity->getNearbyRobots().size() == 2;
        for (auto robotIndex : opportunity->getNearbyRobots())
        {
            auto ctl = dynamic_cast<CoopFixed2Controller *>(gRobots[robotIndex]->getController());
            ctl->setCanMove(canMove);
        }
        opportunity->clearNearbyRobots();
    }
}

void CoopFixed2WorldObserver::teleportRobots(std::set<int> robotsToTeleport) const
{
    for (auto robotIndex : robotsToTeleport)
    {
        gRobots[robotIndex]->unregisterRobot();
        gRobots[robotIndex]->findRandomLocation(gAgentsInitAreaX, gAgentsInitAreaX + gAgentsInitAreaWidth,
                                                gAgentsInitAreaY, gAgentsInitAreaY + gAgentsInitAreaHeight);
        gRobots[robotIndex]->registerRobot();
    }
}


void CoopFixed2WorldObserver::updateEnvironment()
{

}

void CoopFixed2WorldObserver::updateMonitoring()
{
    if ( (_generationCount+1) % CoopFixed2SharedData::gGenerationLog == 0 && CoopFixed2SharedData::gTakeVideo)
    {
        std::string name = "gen_" + std::to_string(_generationCount);
        saveCustomScreenshot(name);
    }
}

void CoopFixed2WorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int nbTrueRobots = gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots;
    
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<int> index(nbTrueRobots);
    
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        auto ctl = dynamic_cast<CoopFixed2Controller*>(gWorld->getRobot(iRobot)->getController());
        fitnesses[iRobot] = ctl->getFitness();
        index[iRobot] = iRobot;
    }
    
    std::sort(index.begin(), index.end(), [&](int i, int j){ return fitnesses[i]<fitnesses[j]; });
    
//    printf("Robots sorted by decreasing fitness\n");
//    for (int i = 0; i < nbTrueRobots; i++)
//        printf("Robot #%.2d is %.2d with %.2lf\n", i, index[nbTrueRobots-i-1], fitnesses[index[nbTrueRobots-i-1]]);
    
    double minFit = fitnesses[index[0]];
    double maxFit = fitnesses[index[nbTrueRobots-1]];
    double medFit = fitnesses[index[nbTrueRobots/2]];
    double lowQuartFit = fitnesses[index[nbTrueRobots/4]];
    double highQuartFit = fitnesses[index[3*nbTrueRobots/4]];
    double avgFit = std::accumulate(fitnesses.begin(), fitnesses.end(), 0.0)/(double)nbTrueRobots;
    double stddevFit = -1;
    for (auto& fitness: fitnesses)
        fitness = (fitness-avgFit)*(fitness-avgFit);
    stddevFit = pow(std::accumulate(fitnesses.begin(), fitnesses.end(), 0.0)/(double)nbTrueRobots, 0.5);
    
    std::stringstream genLog;
    
    genLog << std::setprecision(5);
    genLog << _generationCount+1 << "\t";
    genLog << nbTrueRobots << "\t";
    genLog << minFit << "\t";
    genLog << maxFit << "\t";
    genLog << avgFit << "\t";
    genLog << lowQuartFit << "\t";
    genLog << medFit << "\t";
    genLog << highQuartFit << "\t";
    genLog << stddevFit << "\n";
    
    _fitnessLogManager->write(genLog.str());
    _fitnessLogManager->flush();
    
    if ( (_generationCount+1) % CoopFixed2SharedData::gGenerationLog == 0)
    {
        // log all genomes of each detailed generation, by decreasing fitness
        std::stringstream genomes;
        genomes << _generationCount << " ";
        genomes << nbTrueRobots << "\n";
        for (int iRobot = nbTrueRobots-1; iRobot >= 0; iRobot--)
        {
            auto ctl = dynamic_cast<CoopFixed2Controller *>(gWorld->getRobot(index[iRobot])->getController());
            CoopFixed2Controller::genome gen = ctl->getGenome();
            genomes << gen.second << " ";
            genomes << gen.first.size() << " ";
            for (auto gene: gen.first)
                genomes << gene << " ";
            genomes << "\n\n";
        }
        _genomeLogManager->write(genomes.str());
        _genomeLogManager->flush();
    }
    
    // display lightweight logs for easy-parsing
    std::cout << "log," << (gWorld->getIterations()/CoopFixed2SharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
    
}

void CoopFixed2WorldObserver::addRobotToTeleport(const int i)
{
    _robotsToTeleport.insert(i);
}
