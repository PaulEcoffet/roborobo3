/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MonoRobot/include/MonoRobotWorldObserver.h"
#include "MonoRobot/include/MonoRobotController.h"
#include "World/World.h"
#include "Utilities/Misc.h"

#include <cfloat>
#include <random>
#include <algorithm>

MonoRobotWorldObserver::MonoRobotWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&MonoRobotSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&MonoRobotSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&MonoRobotSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&MonoRobotSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&MonoRobotSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&MonoRobotSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&MonoRobotSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&MonoRobotSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&MonoRobotSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&MonoRobotSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&MonoRobotSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&MonoRobotSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&MonoRobotSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&MonoRobotSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&MonoRobotSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&MonoRobotSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&MonoRobotSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&MonoRobotSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&MonoRobotSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&MonoRobotSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&MonoRobotSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&MonoRobotSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&MonoRobotSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&MonoRobotSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&MonoRobotSharedData::gSigma,false);
    
    gProperties.checkAndGetPropertyValue("gEvaluationsPerGeneration", &MonoRobotSharedData::gEvaluationsPerGeneration, false);
    
    gProperties.checkAndGetPropertyValue("gGenerationLog", &MonoRobotSharedData::gGenerationLog, false);
    gProperties.checkAndGetPropertyValue("gTakeVideo", &MonoRobotSharedData::gTakeVideo, false);

    gProperties.checkAndGetPropertyValue("gTotalEffort", &MonoRobotSharedData::gTotalEffort, false);
    
    gProperties.checkAndGetPropertyValue("gBorderSize", &MonoRobotSharedData::gBorderSize, true);
    gProperties.checkAndGetPropertyValue("gZoneHeight", &MonoRobotSharedData::gZoneHeight, true);
    gProperties.checkAndGetPropertyValue("gZoneWidth", &MonoRobotSharedData::gZoneWidth, true);
    gProperties.checkAndGetPropertyValue("gNbLines", &MonoRobotSharedData::gNbLines, true);
    gProperties.checkAndGetPropertyValue("gNbRows", &MonoRobotSharedData::gNbRows, true);
    
    gProperties.checkAndGetPropertyValue("gConstantA", &MonoRobotSharedData::gConstantA, true);
    gProperties.checkAndGetPropertyValue("gConstantK", &MonoRobotSharedData::gConstantK, true);
    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _evaluationItCount = 0;
    _generationCount = 0;
    _evaluationCount = 0;
    
    // * Logfile
    
    std::string statsLogFilename = gLogDirectoryname + "/observer.txt";
    _statsLogManager = new LogManager(statsLogFilename);
    _statsLogManager->write("GEN\tPOP\tMINFIT\tMAXFIT\tAVGFIT\tQ1FIT\tQ2FIT\tQ3FIT\tSTDDEV\n");
    _statsLogManager->flush();
    
    std::string genomeLogFilename = gLogDirectoryname + "/genome.txt";
    _genomeLogManager = new LogManager(genomeLogFilename);
    
    std::string effortLogFilename = gLogDirectoryname + "/efforts.txt";
    _effortLogManager = new LogManager(effortLogFilename);
    _effortLogManager->write("Gen\tIter\tID\tCoop\n");

}

MonoRobotWorldObserver::~MonoRobotWorldObserver()
{
    delete _statsLogManager;
    delete _genomeLogManager;
}

void MonoRobotWorldObserver::reset()
{
    resetObjects();
}

void MonoRobotWorldObserver::resetObjects()
{
    resetLandmarks();
}

void MonoRobotWorldObserver::resetLandmarks()
{

}

// Reset the environment, and perform an evolution step if it's time
void MonoRobotWorldObserver::stepEvaluation( bool __newGeneration )
{
    // Save fitness values and genomes before the reset
    double totalFitness = 0;
    std::vector<double> fitnesses(gNbOfRobots);
    std::vector<MonoRobotController::genome> genomes(gNbOfRobots);
    std::vector<int> newGenomePick(gNbOfRobots);
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
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
    for (int iObject = 0; iObject < gNbOfPhysicalObjects; iObject++)
    {
        PhysicalObject *object = gPhysicalObjects[iObject];
        int gridBox = iObject/8; // 8 objects per box
        int line = gridBox/MonoRobotSharedData::gNbRows;
        int row = gridBox%MonoRobotSharedData::gNbRows;
        int xMin = MonoRobotSharedData::gBorderSize + row * (MonoRobotSharedData::gZoneWidth + MonoRobotSharedData::gBorderSize) + 3*gPhysicalObjectDefaultRadius;
        int yMin = MonoRobotSharedData::gBorderSize + line * (MonoRobotSharedData::gZoneHeight + MonoRobotSharedData::gBorderSize) + 3*gPhysicalObjectDefaultRadius;
        int xMax = xMin + MonoRobotSharedData::gZoneWidth - 6*gPhysicalObjectDefaultRadius;
        int yMax = yMin + MonoRobotSharedData::gZoneHeight - 6*gPhysicalObjectDefaultRadius;
        object->findRandomLocation(xMin, xMax, yMin, yMax);
        object->registerObject();
    }
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        Robot *robot = gWorld->getRobot(iRobot);
        robot->reset();
        // super specific stuff here
        int gridBox = iRobot; // 1 robot per box
        int line = gridBox/MonoRobotSharedData::gNbRows;
        int row = gridBox%MonoRobotSharedData::gNbRows;
        int xMin = MonoRobotSharedData::gBorderSize + row * (MonoRobotSharedData::gZoneWidth + MonoRobotSharedData::gBorderSize);
        int yMin = MonoRobotSharedData::gBorderSize + line * (MonoRobotSharedData::gZoneHeight + MonoRobotSharedData::gBorderSize);
        robot->findRandomLocation(xMin, xMin + MonoRobotSharedData::gZoneWidth, yMin, yMin + MonoRobotSharedData::gZoneHeight);
        robot->registerRobot();
    }
    
    
    // update genomes (we do it here because Robot::reset() also resets genomes)
    
    // We're doing several evaluation runs per generation
    
    if (__newGeneration)
    {
        // Create a new generation via selection/mutation
        
        // O(1) fitness-proportionate selection
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
        {
            bool done = false;
            int pick = -1;
            while (done == false) {
                pick = rand()%gNbOfRobots;
                double draw = ranf()*totalFitness;
                if (draw <= fitnesses[pick]) // choose this robot
                    done = true;
            }
            newGenomePick[iRobot] = pick;
        }
        
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
        {
            Robot *robot = gWorld->getRobot(iRobot);
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
            ctl->loadNewGenome(genomes[newGenomePick[iRobot]], true);
        }
    }
    else // We just need to give each robot their own genome and fitness back
    {
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
            Robot *robot = gWorld->getRobot(iRobot);
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
            ctl->loadNewGenome(genomes[iRobot], false);
            ctl->increaseFitness(fitnesses[iRobot]);
        }
    }
    
    // Reset the environment parameters
    resetObjects();
}

void MonoRobotWorldObserver::step()
{
    // Reset objects a few times per generation
    if ((_evaluationItCount+1)%(MonoRobotSharedData::gEvaluationTime/MonoRobotSharedData::gNumberOfPeriods) == 0)
    {
        resetObjects();
    }
    
    // switch to next generation.
    if( _evaluationItCount == MonoRobotSharedData::gEvaluationTime - 1 )
    {
        if (_evaluationCount == MonoRobotSharedData::gEvaluationsPerGeneration - 1)
        {
            monitorPopulation();
            stepEvaluation(true);
            _evaluationCount = 0;
            _generationCount++;
        }
        else
        {
            stepEvaluation(false);
            _evaluationCount++;
        }
        // update iterations and generations counters
        _evaluationItCount = 0;
    }
    else
    {
        _evaluationItCount++;
    }
    
    updateEnvironment();
	updateMonitoring();
}


void MonoRobotWorldObserver::updateEnvironment()
{

}

void MonoRobotWorldObserver::updateMonitoring()
{
    if ( (_generationCount+1) % MonoRobotSharedData::gGenerationLog == 0)
    {
        // Log agent effort values
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
        {
            MonoRobotController *ctl = static_cast<MonoRobotController *>(gWorld->getRobot(iRobot)->getController());
            std::stringstream effort;
            effort << _generationCount << "\t";
            effort << _evaluationItCount << "\t";
            effort << iRobot << "\t";
            effort << ctl->getCooperationLevel() << "\n";
            _effortLogManager->write(effort.str());
        }
        if (MonoRobotSharedData::gTakeVideo)
        {
            std::string name = "gen_" + std::to_string(_generationCount);
            saveCustomScreenshot(name);
        }
    }
}

void MonoRobotWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    std::vector<double> fitnesses(gNbOfRobots);
	std::vector<int> index(gNbOfRobots);
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        MonoRobotController *ctl = dynamic_cast<MonoRobotController*>(gWorld->getRobot(iRobot)->getController());
        fitnesses[iRobot] = ctl->getFitness();
		index[iRobot] = iRobot;
    }
    
    std::sort(index.begin(), index.end(), [&](int i, int j){ return fitnesses[i]<fitnesses[j]; });

//    MonoRobotController *ctl = dynamic_cast<MonoRobotController*>(gWorld->getRobot(index[gNbOfRobots-1])->getController());
//    double avg = -1;
//    std::vector<double> megaEfforts = ctl->getMegaEfforts();
//    double totEff = std::accumulate(megaEfforts.begin(), megaEfforts.end(), 0.0);
//    printf("[DEBUG] Robot %d, lifetime efforts\n", index[gNbOfRobots-1]);
//	if (megaEfforts.size() > 0)
//        avg = totEff/(double)megaEfforts.size();
//	printf("[DEBUG] Total fitness %.3lf, average fitness %.3lf, total effort %.3lf, average effort %.3lf, active time %lu\n", fitnesses[index[gNbOfRobots-1]], fitnesses[index[gNbOfRobots-1]]/megaEfforts.size(), totEff, avg, megaEfforts.size());

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
    
    _statsLogManager->write(genLog.str());
    _statsLogManager->flush();
    
    // log the best genome of each detailed generation
    if ( (_generationCount+1) % MonoRobotSharedData::gGenerationLog == 0)
    {
        // log all genomes of each detailed generation, by decreasing fitness
        std::stringstream genomes;
        genomes << _generationCount << " ";
        genomes << gNbOfRobots << "\n";
        for (int iRobot = gNbOfRobots-1; iRobot >= 0; iRobot--)
        {
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(gWorld->getRobot(index[iRobot])->getController());
            MonoRobotController::genome gen = ctl->getGenome();
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
    std::cout << "log," << (gWorld->getIterations()/MonoRobotSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
        
    // Logging, population-level: alive
//    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(gNbOfRobots) + "\n";
//    gLogManager->write(sLog);
//    gLogManager->flush();
    
}
