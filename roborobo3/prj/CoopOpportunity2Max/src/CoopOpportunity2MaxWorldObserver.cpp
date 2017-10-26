/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxWorldObserver.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxController.h"
#include "World/World.h"
#include "Utilities/Misc.h"

#include <cfloat>
#include <random>
#include "World/MovingObject.h"

CoopOpportunity2MaxWorldObserver::CoopOpportunity2MaxWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&CoopOpportunity2MaxSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&CoopOpportunity2MaxSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&CoopOpportunity2MaxSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&CoopOpportunity2MaxSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&CoopOpportunity2MaxSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&CoopOpportunity2MaxSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&CoopOpportunity2MaxSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&CoopOpportunity2MaxSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&CoopOpportunity2MaxSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&CoopOpportunity2MaxSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&CoopOpportunity2MaxSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&CoopOpportunity2MaxSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&CoopOpportunity2MaxSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&CoopOpportunity2MaxSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&CoopOpportunity2MaxSharedData::gControllerType,true);

    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&CoopOpportunity2MaxSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&CoopOpportunity2MaxSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&CoopOpportunity2MaxSharedData::gSigma,false);
    
    gProperties.checkAndGetPropertyValue("gTotalEffort", &CoopOpportunity2MaxSharedData::gTotalEffort, false);
    
    gProperties.checkAndGetPropertyValue("gFakeCoopValue", &CoopOpportunity2MaxSharedData::gFakeCoopValue, false);
    gProperties.checkAndGetPropertyValue("gNbFakeRobots", &CoopOpportunity2MaxSharedData::gNbFakeRobots, false);
    
    gProperties.checkAndGetPropertyValue("gFixedEffort", &CoopOpportunity2MaxSharedData::gFixedEffort, false);
    gProperties.checkAndGetPropertyValue("gFixedEffortValue", &CoopOpportunity2MaxSharedData::gFixedEffortValue, false);
    
    gProperties.checkAndGetPropertyValue("gGenerationLog", &CoopOpportunity2MaxSharedData::gGenerationLog, false);
    gProperties.checkAndGetPropertyValue("gTakeVideo", &CoopOpportunity2MaxSharedData::gTakeVideo, false);
    gProperties.checkAndGetPropertyValue("gLogGenome",&CoopOpportunity2MaxSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&CoopOpportunity2MaxSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gConstantA", &CoopOpportunity2MaxSharedData::gConstantA, true);
    gProperties.checkAndGetPropertyValue("gConstantK", &CoopOpportunity2MaxSharedData::gConstantK, true);

    
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

CoopOpportunity2MaxWorldObserver::~CoopOpportunity2MaxWorldObserver()
{
    delete _fitnessLogManager;
    delete _genomeLogManager;
}

void CoopOpportunity2MaxWorldObserver::reset()
{
    
}

// Reset the environment, and perform an evolution step
void CoopOpportunity2MaxWorldObserver::stepEvaluation()
{
    // Save fitness values and genomes before the reset
    double totalFitness = 0;
    int nbTrueRobots = gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots;
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<CoopOpportunity2MaxController::genome> genomes(nbTrueRobots);
    std::vector<int> newGenomePick(gNbOfRobots);
    // don't consider the last gNbFakeRobots robots (they have fixed cooperation levels)
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        CoopOpportunity2MaxController *ctl = dynamic_cast<CoopOpportunity2MaxController *>(robot->getController());
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
        CoopOpportunity2MaxController *ctl = dynamic_cast<CoopOpportunity2MaxController *>(robot->getController());
        ctl->loadNewGenome(genomes[newGenomePick[iRobot]]);
    }
//    for (int iRobot = 0; iRobot < 40; iRobot++)
//        printf("Robot %.2d was picked %.2lf%% of the time and had proba %.2lf%%\n", iRobot, (double)nbPicks[iRobot]/50.0*100.0, fitnesses[iRobot]/totalFitness*100.0);
}

void CoopOpportunity2MaxWorldObserver::step()
{
    // switch to next generation.
    if( _generationItCount == CoopOpportunity2MaxSharedData::gEvaluationTime - 1 )
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


void CoopOpportunity2MaxWorldObserver::updateEnvironment()
{

}

void CoopOpportunity2MaxWorldObserver::updateMonitoring()
{
    if ( (_generationCount+1) % CoopOpportunity2MaxSharedData::gGenerationLog == 0 && CoopOpportunity2MaxSharedData::gTakeVideo)
    {
        std::string name = "gen_" + std::to_string(_generationCount);
        saveCustomScreenshot(name);
    }
}

void CoopOpportunity2MaxWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int nbTrueRobots = gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots;
    
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<int> index(nbTrueRobots);
    
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        CoopOpportunity2MaxController *ctl = dynamic_cast<CoopOpportunity2MaxController*>(gWorld->getRobot(iRobot)->getController());
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
    
    if ( (_generationCount+1) % CoopOpportunity2MaxSharedData::gGenerationLog == 0)
    {
        // log all genomes of each detailed generation, by decreasing fitness
        std::stringstream genomes;
        genomes << _generationCount << " ";
        genomes << nbTrueRobots << "\n";
        for (int iRobot = nbTrueRobots-1; iRobot >= 0; iRobot--)
        {
            CoopOpportunity2MaxController *ctl = dynamic_cast<CoopOpportunity2MaxController *>(gWorld->getRobot(index[iRobot])->getController());
            CoopOpportunity2MaxController::genome gen = ctl->getGenome();
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
    std::cout << "log," << (gWorld->getIterations()/CoopOpportunity2MaxSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
    
}
