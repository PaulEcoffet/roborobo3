/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MonoRobot/include/MonoRobotWorldObserver.h"
#include "MonoRobot/include/MonoRobotController.h"
#include "World/World.h"

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
    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _generationItCount = -1;
    _generationCount = -1;
    
    _startObjectOffset = rand()%2;
    
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
    
}

void MonoRobotWorldObserver::step()
{
    _generationItCount++;
    
    // switch to next generation.
    if( _generationItCount == MonoRobotSharedData::gEvaluationTime )
    {        
        // Perform an evolution step on robots (give them new genomes and mutate them), and
        // reset their positions and the objects
        
        // Evolution stuff
        
        double totalFitness = 0;
        std::vector<double> fitnesses(gNbOfRobots);
        std::vector<genome> genomes(gNbOfRobots);
		std::vector<int> newGenomePick(gNbOfRobots);
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
        {
            Robot *robot = gWorld->getRobot(iRobot);
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
            fitnesses[iRobot] = ctl->getFitness();
            totalFitness += ctl->getFitness();
            genomes[iRobot] = ctl->getGenome();
        }
			
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
                
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
        
        _startObjectOffset = rand()%2;
    }
    
    updateMonitoring();
    
    updateEnvironment();
}


void MonoRobotWorldObserver::updateEnvironment()
{

}

void MonoRobotWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    //if( gWorld->getIterations() % MonoRobotSharedData::gEvaluationTime == 1 || gWorld->getIterations() % MonoRobotSharedData::gEvaluationTime == MonoRobotSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    // log at end of generation.
    if( _generationItCount == MonoRobotSharedData::gEvaluationTime - 1)
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( MonoRobotSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( MonoRobotSharedData::gEvaluationTime * MonoRobotSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / MonoRobotSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( MonoRobotSharedData::gEvaluationTime * MonoRobotSharedData::gSnapshotsFrequency ) == MonoRobotSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / MonoRobotSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
	}
}

void MonoRobotWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    std::vector<double> fitnesses(gNbOfRobots);
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        MonoRobotController *ctl = dynamic_cast<MonoRobotController*>(gWorld->getRobot(iRobot)->getController());
        fitnesses[iRobot] = ctl->getFitness();
    }
    
    std::sort(fitnesses.begin(), fitnesses.end());
    
    double minFit = fitnesses[0];
    double maxFit = fitnesses[gNbOfRobots-1];
    double medFit = fitnesses[gNbOfRobots/2];
    double lowQuartFit = fitnesses[gNbOfRobots/4];
    double highQuartFit = fitnesses[3*gNbOfRobots/4];
    double avgFit = std::accumulate(fitnesses.begin(), fitnesses.end(), 0)/(double)gNbOfRobots;
    double stddevFit = -1;
    for (auto& fitness: fitnesses)
        fitness = (fitness-avgFit)*(fitness-avgFit);
    stddevFit = pow(std::accumulate(fitnesses.begin(), fitnesses.end(), 0)/(double)gNbOfRobots, 0.5);
    
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
