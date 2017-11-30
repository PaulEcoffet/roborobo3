/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MovingEE/include/MovingEEWorldObserver.h"
#include "MovingEE/include/MovingEEController.h"
#include "World/World.h"

#include <float.h> // for DBL_MAX

MovingEEWorldObserver::MovingEEWorldObserver( World* world ) : TemplateEEWorldObserver( world )
{
    // superclass constructor called before
}

MovingEEWorldObserver::~MovingEEWorldObserver()
{
    // superclass constructor called before
}

void MovingEEWorldObserver::stepPre()
{
    TemplateEEWorldObserver::stepPre();
    
    /*
    if( gWorld->getIterations() > 0 && gWorld->getIterations() % TemplateEESharedData::gEvaluationTime == 0 )
    {
        std::cout << "[DEBUG] new generation.\n";
    }
    */

}

void MovingEEWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int activeCount = 0;
    double sumOfFitnesses = 0;
    double minFitness = DBL_MAX;
    double maxFitness = -DBL_MAX;
    
    for ( int i = 0 ; i != gNbOfRobots ; i++ )
    {
        MovingEEController *ctl = dynamic_cast<MovingEEController*>(gWorld->getRobot(i)->getController());
        
        if ( ctl->getWorldModel()->isAlive() == true )
        {
            activeCount++;
            sumOfFitnesses += ctl->getFitness() ;
            if ( ctl->getFitness() < minFitness )
                minFitness = ctl->getFitness();
            if ( ctl->getFitness() > maxFitness )
                maxFitness = ctl->getFitness();
        }
    }
    
    if ( gVerbose && localVerbose )
    {
        std::cout << "[gen:" << (gWorld->getIterations()/TemplateEESharedData::gEvaluationTime) << ";it:" << gWorld->getIterations() << ";pop:" << activeCount << ";avgFitness:" << sumOfFitnesses/activeCount << "]\n";
    }
    
    // display lightweight logs for easy-parsing
    std::cout << "log," << (gWorld->getIterations()/TemplateEESharedData::gEvaluationTime) << "," << gWorld->getIterations() << "," << activeCount << "," << minFitness << "," << maxFitness << "," << sumOfFitnesses/activeCount << "\n";

    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}
