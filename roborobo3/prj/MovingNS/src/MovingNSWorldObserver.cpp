/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MovingNS/include/MovingNSWorldObserver.h"
#include "MovingNS/include/MovingNSController.h"
#include "World/World.h"
#include "Utilities/Misc.h"

#include <cfloat>
#include <random>

MovingNSWorldObserver::MovingNSWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&MovingNSSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&MovingNSSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&MovingNSSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&MovingNSSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&MovingNSSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&MovingNSSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&MovingNSSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&MovingNSSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&MovingNSSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&MovingNSSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&MovingNSSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&MovingNSSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&MovingNSSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&MovingNSSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&MovingNSSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&MovingNSSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&MovingNSSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&MovingNSSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&MovingNSSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&MovingNSSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&MovingNSSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&MovingNSSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&MovingNSSharedData::gSigma,false);
    
    gProperties.checkAndGetPropertyValue("gTotalEffort", &MovingNSSharedData::gTotalEffort, false);
    
    gProperties.checkAndGetPropertyValue("gFakeCoopValue", &MovingNSSharedData::gFakeCoopValue, false);
    gProperties.checkAndGetPropertyValue("gNbFakeRobots", &MovingNSSharedData::gNbFakeRobots, false);
    
    gProperties.checkAndGetPropertyValue("gFixedEffort", &MovingNSSharedData::gFixedEffort, false);
    gProperties.checkAndGetPropertyValue("gFixedEffortValue", &MovingNSSharedData::gFixedEffortValue, false);
    
    gProperties.checkAndGetPropertyValue("gGenerationLog", &MovingNSSharedData::gGenerationLog, false);
    gProperties.checkAndGetPropertyValue("gTakeVideo", &MovingNSSharedData::gTakeVideo, false);
    gProperties.checkAndGetPropertyValue("gLogGenome",&MovingNSSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&MovingNSSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gConstantA", &MovingNSSharedData::gConstantA, true);
    gProperties.checkAndGetPropertyValue("gConstantK", &MovingNSSharedData::gConstantK, true);

    
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
    
    time_t now = time(NULL);
	char chartime[19] = "";
	strftime (chartime, 19,"%Y%m%d_%Hh%Mm%Ss", localtime(&now));
	std::string stime = chartime;
    std::string fitnessLogFilename = gLogDirectoryname + "/" + stime + "_observer.txt";
    _fitnessLogManager = new LogManager(fitnessLogFilename);
    _fitnessLogManager->write("GEN\tPOP\tMINFIT\tMAXFIT\tAVGFIT\tQ1FIT\tQ2FIT\tQ3FIT\tSTDDEV\n");
    
    std::string genomeLogFilename = gLogDirectoryname + "/" + stime + "_genome.txt";
    _genomeLogManager = new LogManager(genomeLogFilename);
}

MovingNSWorldObserver::~MovingNSWorldObserver()
{
    delete _fitnessLogManager;
    delete _genomeLogManager;
}

void MovingNSWorldObserver::reset()
{
    
}

// Reset the environment, and perform an evolution step
void MovingNSWorldObserver::stepEvaluation()
{
    // Save fitness values and genomes before the reset
    double totalFitness = 0;
    int nbTrueRobots = gNbOfRobots - MovingNSSharedData::gNbFakeRobots;
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<MovingNSController::genome> genomes(nbTrueRobots);
    std::vector<int> newGenomePick(gNbOfRobots);
    // don't consider the last gNbFakeRobots robots (they have fixed cooperation levels)
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
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
        MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
        ctl->loadNewGenome(genomes[newGenomePick[iRobot]]);
    }
//    for (int iRobot = 0; iRobot < 40; iRobot++)
//        printf("Robot %.2d was picked %.2lf%% of the time and had proba %.2lf%%\n", iRobot, (double)nbPicks[iRobot]/50.0*100.0, fitnesses[iRobot]/totalFitness*100.0);
}

void MovingNSWorldObserver::step()
{
    // switch to next generation.
    if( _generationItCount == MovingNSSharedData::gEvaluationTime - 1 )
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


void MovingNSWorldObserver::updateEnvironment()
{

}

void MovingNSWorldObserver::updateMonitoring()
{
    if ( (_generationCount+1) % MovingNSSharedData::gGenerationLog == 0 && MovingNSSharedData::gTakeVideo)
    {
        std::string name = "gen_" + std::to_string(_generationCount);
        saveCustomScreenshot(name);
    }
}

void MovingNSWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int nbTrueRobots = gNbOfRobots - MovingNSSharedData::gNbFakeRobots;
    
    std::vector<double> fitnesses(nbTrueRobots);
    std::vector<int> index(nbTrueRobots);
    
    for (int iRobot = 0; iRobot < nbTrueRobots; iRobot++)
    {
        MovingNSController *ctl = dynamic_cast<MovingNSController*>(gWorld->getRobot(iRobot)->getController());
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
    
    if ( (_generationCount+1) % MovingNSSharedData::gGenerationLog == 0)
    {
        // log all genomes of each detailed generation, by decreasing fitness
        std::stringstream genomes;
        genomes << _generationCount << " ";
        genomes << nbTrueRobots << "\n";
        for (int iRobot = nbTrueRobots-1; iRobot >= 0; iRobot--)
        {
            MovingNSController *ctl = dynamic_cast<MovingNSController *>(gWorld->getRobot(index[iRobot])->getController());
            MovingNSController::genome gen = ctl->getGenome();
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
    std::cout << "log," << (gWorld->getIterations()/MovingNSSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
    
}
