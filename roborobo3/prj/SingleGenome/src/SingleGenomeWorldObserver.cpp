/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "SingleGenome/include/SingleGenomeWorldObserver.h"
#include "SingleGenome/include/SingleGenomeController.h"
#include "World/World.h"
#include "Utilities/Misc.h"

#include <cfloat>
#include <random>

SingleGenomeWorldObserver::SingleGenomeWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&SingleGenomeSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&SingleGenomeSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&SingleGenomeSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&SingleGenomeSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&SingleGenomeSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&SingleGenomeSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&SingleGenomeSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&SingleGenomeSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&SingleGenomeSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&SingleGenomeSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&SingleGenomeSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&SingleGenomeSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&SingleGenomeSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&SingleGenomeSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&SingleGenomeSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&SingleGenomeSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&SingleGenomeSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&SingleGenomeSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&SingleGenomeSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&SingleGenomeSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&SingleGenomeSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&SingleGenomeSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&SingleGenomeSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&SingleGenomeSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&SingleGenomeSharedData::gSigma,false);
    
    gProperties.checkAndGetPropertyValue("gTotalEffort", &SingleGenomeSharedData::gTotalEffort, true);
    
    gProperties.checkAndGetPropertyValue("gGenerationLog", &SingleGenomeSharedData::gGenerationLog, false);
    gProperties.checkAndGetPropertyValue("gTakeVideo", &SingleGenomeSharedData::gTakeVideo, false);
    
    gProperties.checkAndGetPropertyValue("gFakeRobotsPerObject", &SingleGenomeSharedData::gFakeRobotsPerObject, false);
    gProperties.checkAndGetPropertyValue("gFakeCoopValue", &SingleGenomeSharedData::gFakeCoopValue, false);
        
    gProperties.checkAndGetPropertyValue("gConstantA", &SingleGenomeSharedData::gConstantA, true);
    gProperties.checkAndGetPropertyValue("gConstantK", &SingleGenomeSharedData::gConstantK, true);
    
    gProperties.checkAndGetPropertyValue("gGenomeFilename", &SingleGenomeSharedData::gGenomeFilename, true);

    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * Logfiles
    
    std::string fitnessLogFilename = gLogDirectoryname + "/observer.txt";
    _fitnessLogManager = new LogManager(fitnessLogFilename);
    _fitnessLogManager->write("GEN\tPOP\tMINFIT\tMAXFIT\tAVGFIT\tQ1FIT\tQ2FIT\tQ3FIT\tSTDDEV\n");
    _fitnessLogManager->flush();
    
    std::string coopLogFilename = gLogDirectoryname + "/coop_stats.txt";
    _coopLogManager = new LogManager(coopLogFilename);
    _coopLogManager->write("Genome\tfkeRob\tfkeCoop\tRep\tIter\tID\tnbRob\tCoop\n");
}

SingleGenomeWorldObserver::~SingleGenomeWorldObserver()
{
    delete _fitnessLogManager;
    delete _coopLogManager;
}

void SingleGenomeWorldObserver::reset()
{
    // * Load all the genomes that we are going to use
    
    std::string filename = "config/"+SingleGenomeSharedData::gGenomeFilename;
    std::ifstream genomeFile(filename);
    if (genomeFile.fail())
    {
        printf("[CRITICAL] Could not read genome file (%s). Exiting.\n", filename.c_str());
        exit(-1);
    }
    
    int nbGenomes;
    genomeFile >> nbGenomes;
    for (int iGenome = 0; iGenome < nbGenomes; iGenome++)
    {
        SingleGenomeController::genome gen;
        genomeFile >> gen.second; // sigma value
        int nbGenes;
        genomeFile >> nbGenes;
        // Note: we can't test that the genome has the right number of genes here
        for (int iGene = 0; iGene < nbGenes; iGene++) {
            double v;
            genomeFile >> v;
            gen.first.push_back(v);
        }
        _genomes.push_back(gen);
    }
    
    // Fill the fake coop values
    for (int i = 0; i < SingleGenomeSharedData::gFakeCoopSteps; i++)
        _fakeCoopValues.push_back(((double)i)/((double)(SingleGenomeSharedData::gFakeCoopSteps-1))*SingleGenomeSharedData::gFakeCoopValue);
    
    // * iteration and generation counters
    
    _generationItCount = 0;
    _generationCount = 0;
    
    gMaxIt = SingleGenomeSharedData::gEvaluationTime * nbGenomes * SingleGenomeSharedData::gNbReplicas * SingleGenomeSharedData::gFakeCoopSteps * (SingleGenomeSharedData::gFakeRobotsPerObject +1);
    
    // Parameter values
    _genome = 0;
    _replica = 0;
    _fakeCoop = 0;
    _nbFakeRobots = 0;
    loadGenomes();

}

void SingleGenomeWorldObserver::loadGenomes()
{
    printf("Loading genome %d in the robots...\n", _genome);
    for (int iRobot = 0; iRobot < gInitialNumberOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        SingleGenomeController *ctl = dynamic_cast<SingleGenomeController *>(robot->getController());
        ctl->loadNewGenome(_genomes[_genome]);
    }
}

// Reset everything and perform the next evaluation run
void SingleGenomeWorldObserver::stepEvaluation()
{
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
    
    // move on to the:
    // -next genome if we're done with the current one
    // -next set of parameters if we're done with the current one
    // -next replica otherwise
    // We have a "-1" each time because we run another step afterwards
    if (_replica < SingleGenomeSharedData::gNbReplicas-1)
    {
        _replica++;
        return;
    }
    _replica = 0;
    if (_nbFakeRobots <= SingleGenomeSharedData::gFakeRobotsPerObject-1) // include the last value
    {
        _nbFakeRobots++;
        return;
    }
    _nbFakeRobots = 0;
    if (_fakeCoop < SingleGenomeSharedData::gFakeCoopSteps-1)
    {
        _fakeCoop++;
        return;
    }
    _fakeCoop = 0;
    
    _genome++; // we've done all the parameters, switch to next genome
    loadGenomes();
}

void SingleGenomeWorldObserver::step()
{
//    printf("Genome %d replica %d fakeRobots %d fakeCoop %d iter %d\n", _genome, _replica, _nbFakeRobots, _fakeCoop, _generationItCount);
    // switch to next generation.
    if( _generationItCount == SingleGenomeSharedData::gEvaluationTime - 1 )
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


void SingleGenomeWorldObserver::updateEnvironment()
{

}

void SingleGenomeWorldObserver::updateMonitoring()
{
    if (SingleGenomeSharedData::gTakeVideo)
    {
        std::string name = "gen_" + std::to_string(_generationCount);
        saveCustomScreenshot(name);
    }
}

void SingleGenomeWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    std::vector<double> fitnesses(gNbOfRobots);
    std::vector<int> index(gNbOfRobots);
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        SingleGenomeController *ctl = dynamic_cast<SingleGenomeController*>(gWorld->getRobot(iRobot)->getController());
        fitnesses[iRobot] = ctl->getFitness();
        index[iRobot] = iRobot;
    }
    
    std::sort(index.begin(), index.end(), [&](int i, int j){ return fitnesses[i]<fitnesses[j]; });
    
    double minFit = fitnesses[index[0]];
    double maxFit = fitnesses[index[gNbOfRobots-1]];
    double medFit = fitnesses[index[gNbOfRobots/2]];
    double lowQuartFit = fitnesses[index[gNbOfRobots/4]];
    double highQuartFit = fitnesses[index[3*gNbOfRobots/4]];
    double avgFit = std::accumulate(fitnesses.begin(), fitnesses.end(), 0.0)/(double)gNbOfRobots;
    double stddevFit = -1;
    for (auto& fitness: fitnesses)
        fitness = (fitness-avgFit)*(fitness-avgFit);
    stddevFit = pow(std::accumulate(fitnesses.begin(), fitnesses.end(), 0.0)/(double)gNbOfRobots, 0.5);
    
    std::stringstream genLog;
    
    genLog << std::setprecision(5);
    genLog << _generationCount+1 << "\t";
    genLog << gNbOfRobots << "\t";
    genLog << minFit << "\t";
    genLog << maxFit << "\t";
    genLog << avgFit << "\t";
    genLog << lowQuartFit << "\t";
    genLog << medFit << "\t";
    genLog << highQuartFit << "\t";
    genLog << stddevFit << "\n";
    
    _fitnessLogManager->write(genLog.str());
    _fitnessLogManager->flush();
    
    // display lightweight logs for easy-parsing
    std::cout << "log," << (gWorld->getIterations()/SingleGenomeSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
    
}
