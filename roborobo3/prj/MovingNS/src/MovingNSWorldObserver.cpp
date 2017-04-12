/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MovingNS/include/MovingNSWorldObserver.h"
#include "MovingNS/include/MovingNSController.h"
#include "World/World.h"

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
}

MovingNSWorldObserver::~MovingNSWorldObserver()
{
    // nothing to do.
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
    
    double sumOfFitnesses = 0;
    double minFitness = DBL_MAX;
    double maxFitness = -DBL_MAX;
    
    for ( int i = 0 ; i != gNbOfRobots ; i++ )
    {
        MovingNSController *ctl = dynamic_cast<MovingNSController*>(gWorld->getRobot(i)->getController());

            sumOfFitnesses += ctl->getFitness() ;
            if ( ctl->getFitness() < minFitness )
                minFitness = ctl->getFitness();
            if ( ctl->getFitness() > maxFitness )
                maxFitness = ctl->getFitness();
    }
    
    if ( gVerbose && localVerbose )
    {
        std::cout << "[gen:" << (gWorld->getIterations()/MovingNSSharedData::gEvaluationTime) << ";it:" << gWorld->getIterations() << ";pop:" << gNbOfRobots << ";avgFitness:" << sumOfFitnesses/gNbOfRobots << "]\n";
    }
    
    // display lightweight logs for easy-parsing
    std::cout << "log," << (gWorld->getIterations()/MovingNSSharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << gNbOfRobots << "," << minFitness << "," << maxFitness << "," << sumOfFitnesses/gNbOfRobots << "\n";
        
    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(gNbOfRobots) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
    
}
