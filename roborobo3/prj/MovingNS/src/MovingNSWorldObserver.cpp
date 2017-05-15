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
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&MovingNSSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&MovingNSSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&MovingNSSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&MovingNSSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&MovingNSSharedData::gSigma,false);
    
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

MovingNSWorldObserver::~MovingNSWorldObserver()
{
    _logFile.close();
}

void MovingNSWorldObserver::reset()
{
    
}

void MovingNSWorldObserver::step()
{
    _generationItCount++;
 
    // switch to next generation.
    if( _generationItCount == MovingNSSharedData::gEvaluationTime )
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
            MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
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
        
        for (auto object: gPhysicalObjects) {
            object->unregisterObject();
        }
        
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
            Robot *robot = gWorld->getRobot(iRobot);
            robot->unregisterRobot();
        }
        for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++) {
            Robot *robot = gWorld->getRobot(iRobot);
            robot->reset();
            robot->registerRobot();
        }
        
        for (auto object: gPhysicalObjects)
        {
            object->findRandomLocation();
            object->registerObject();
        }

		// update genomes (we do it here because Robot::reset() also resets genomes)
		
		for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
		{
            Robot *robot = gWorld->getRobot(iRobot);
            MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
			ctl->loadNewGenome(genomes[newGenomePick[iRobot]]);
		}
        
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
}


void MovingNSWorldObserver::updateEnvironment()
{

}

void MovingNSWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    //if( gWorld->getIterations() % MovingNSSharedData::gEvaluationTime == 1 || gWorld->getIterations() % MovingNSSharedData::gEvaluationTime == MovingNSSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    // log at end of generation.
    if( _generationItCount == MovingNSSharedData::gEvaluationTime - 1)
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( MovingNSSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( MovingNSSharedData::gEvaluationTime * MovingNSSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / MovingNSSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( MovingNSSharedData::gEvaluationTime * MovingNSSharedData::gSnapshotsFrequency ) == MovingNSSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / MovingNSSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
	}
}

void MovingNSWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    std::vector<double> fitnesses(gNbOfRobots);
    
    for (int iRobot = 0; iRobot < gNbOfRobots; iRobot++)
    {
        MovingNSController *ctl = dynamic_cast<MovingNSController*>(gWorld->getRobot(iRobot)->getController());
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
    std::cout << "log," << (gWorld->getIterations()/MovingNSSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFit << "," << maxFit << "," << avgFit << "\n";
        
    // Logging, population-level: alive
//    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(gNbOfRobots) + "\n";
//    gLogManager->write(sLog);
//    gLogManager->flush();
    
}
