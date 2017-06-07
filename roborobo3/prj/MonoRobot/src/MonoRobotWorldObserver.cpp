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
    
    _generationItCount = -1;
    _generationCount = -1;
    
    // * Logfile
    
    std::string logFilename = gLogDirectoryname + "/observer.txt";
    _logFile.open(logFilename.c_str());
    _logManager = new LogManager();
    _logManager->setLogFile(_logFile);
    _logManager->write("GEN\tPOP\tMINFIT\tMAXFIT\tAVGFIT\tQ1FIT\tQ2FIT\tQ3FIT\tSTDDEV\n");
    _logManager->flush();
}

MonoRobotWorldObserver::~MonoRobotWorldObserver()
{
    _logFile.close();
}

void MonoRobotWorldObserver::reset()
{
    // * Active objects and fake robot
    
    resetObjects();
}

void MonoRobotWorldObserver::resetObjects()
{
    resetLandmarks();
}

void MonoRobotWorldObserver::resetLandmarks()
{
    // put the landmarks in the right position
}

void MonoRobotWorldObserver::stepGeneration()
{
    // Perform an evolution step on robots (give them new genomes and mutate them), and
    // reset their positions and the objects
    
    // Evolution stuff
        
    double totalFitness = 0;
    std::vector<double> fitnesses(gNbOfRobots);
    std::vector<genome> genomes(gNbOfRobots);
    std::vector<int> newGenomePick(gNbOfRobots);
    std::vector<int> timesPicked(gNbOfRobots);
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
        fitnesses[iRobot] = ctl->getFitness();
        totalFitness += ctl->getFitness();
        genomes[iRobot] = ctl->getGenome();
        timesPicked[iRobot] = 0;
    }
    
    
//    // O(1) fitness-proportionate selection
//    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
//    {
//        bool done = false;
//        int pick = -1;
//        while (done == false) {
//            pick = rand()%gNbOfRobots;
//            double draw = ranf()*totalFitness;
//            if (draw <= fitnesses[pick]) // choose this robot
//                done = true;
//        }
//        newGenomePick[iRobot] = pick;
//        timesPicked[pick]++;
//    }
    
    // O(n) fitness-proportionate selection
    // sort fitnesses in decreasing order
    std::vector<int> fitnessIndex(gNbOfRobots);
    for (int i = 0; i < gNbOfRobots; i++)
        fitnessIndex[i] = i;
    std::sort(fitnessIndex.begin(), fitnessIndex.end(), [&](int i, int j){ return fitnesses[i]>fitnesses[j]; });
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        printf("Choosing new genome for robot %d\n", iRobot);
        double choice = ranf()*totalFitness;
        printf("Choice: %lf\n", choice);
        double total = 0;
        int res = 0;
        for (int i = 0; i < gNbOfRobots; i++) {
            total += fitnesses[fitnessIndex[i]];
            printf("Current fitness: %lf, total: %lf\n", fitnesses[fitnessIndex[i]], total);
            if (choice <= total) {
                res = fitnessIndex[i];
                printf("Chose genome %d\n", res);
                break;
            }
        }
        printf("\n");
        newGenomePick[iRobot] = res;
        timesPicked[res]++;
    }
    
    // Debug printing
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
        printf("[DEBUG] Robot %.2d: fitness %.3lf, prob. %.3lf%%, actual %.3lf%%\n", iRobot, fitnesses[iRobot], fitnesses[iRobot]/totalFitness*100.0, (double)timesPicked[iRobot]/(double)gNbOfRobots*100.0);
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
        // super specific stuff here
        int gridBox = iRobot/2;
        int line = gridBox/MonoRobotSharedData::gNbRows;
        int row = gridBox%MonoRobotSharedData::gNbRows;
        int xMin = MonoRobotSharedData::gBorderSize + row * (MonoRobotSharedData::gZoneWidth + MonoRobotSharedData::gBorderSize);
        int yMin = MonoRobotSharedData::gBorderSize + line * (MonoRobotSharedData::gZoneHeight + MonoRobotSharedData::gBorderSize);
        robot->findRandomLocation(xMin, xMin + MonoRobotSharedData::gZoneWidth, yMin, yMin + MonoRobotSharedData::gZoneHeight);
        robot->registerRobot();
    }
    
    
    // update genomes (we do it here because Robot::reset() also resets genomes)
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        Robot *robot = gWorld->getRobot(iRobot);
        MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
        ctl->loadNewGenome(genomes[newGenomePick[iRobot]]);
    }
    
    // Reset the environment parameters
    
    resetObjects();

}

void MonoRobotWorldObserver::step()
{
    _generationItCount++;
    
    // switch to next generation.
    if( _generationItCount == MonoRobotSharedData::gEvaluationTime )
    {
        stepGeneration();
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
}


void MonoRobotWorldObserver::updateEnvironment()
{

}

void MonoRobotWorldObserver::updateMonitoring()
{
    if( _generationItCount == MonoRobotSharedData::gEvaluationTime - 1)
    {
        monitorPopulation();
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
    
    _logManager->write(genLog.str());
    _logManager->flush();
    
    // display lightweight logs for easy-parsing
    std::cout << "log," << (gWorld->getIterations()/MonoRobotSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
        
    // Logging, population-level: alive
//    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(gNbOfRobots) + "\n";
//    gLogManager->write(sLog);
//    gLogManager->flush();
    
}
