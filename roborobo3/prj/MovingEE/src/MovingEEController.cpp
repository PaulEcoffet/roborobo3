/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "MovingEE/include/MovingEEController.h"
#include "MovingEE/include/MovingEEWorldObserver.h"

#include "World/World.h"
#include "Utilities/Misc.h"
#include <math.h>
#include <string>

#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>

using namespace Neural;

MovingEEController::MovingEEController( RobotWorldModel *wm ) : TemplateEEController( wm )
{
    // superclass constructor called before this baseclass constructor.
    resetFitness(); // superconstructor calls parent method.
}

MovingEEController::~MovingEEController()
{
    // superclass destructor automatically called after this baseclass destructor.
}

void MovingEEController::stepController()
{
    TemplateEEController::stepController();
}

void MovingEEController::initController()
{
    TemplateEEController::initController();
}

void MovingEEController::performSelection()
{
    //TemplateEEController::performSelection();
    
    std::pair<int,int> bestId;

    std::map<std::pair<int,int>, float >::iterator fitnessesIt = _fitnessValuesList.begin();

    float bestFitnessValue = (*fitnessesIt).second;
    bestId = (*fitnessesIt).first;

    ++fitnessesIt;

    int nbSimilar = 0;
    
    for ( int i = 1 ; fitnessesIt != _fitnessValuesList.end(); ++fitnessesIt, i++)
    {
        if ( (*fitnessesIt).second >= bestFitnessValue )
        {
            if ( (*fitnessesIt).second > bestFitnessValue )
            {
                bestFitnessValue = (*fitnessesIt).second;
                bestId = (*fitnessesIt).first;
                nbSimilar = 0;
            }
            else
            {
                nbSimilar++;
            }
        }
    }

    if ( nbSimilar > 0 ) // >1 genomes have the same fitness best value. Pick randomly among them
    {
        int count = 0;
        int randomPick = randint() % ( nbSimilar + 1 );
        
        if ( randomPick != 0 ) // not already stored (i.e. not the first one)
        {
            fitnessesIt = _fitnessValuesList.begin();
            for ( int i = 0 ; ; ++fitnessesIt, i++)
            {
                if ( (*fitnessesIt).second == bestFitnessValue )
                {
                    if ( count == randomPick )
                    {
                        bestId = (*fitnessesIt).first;
                        break;
                    }
                    count++;
                }
            }
        }
    }
    
    _birthdate = gWorld->getIterations();
    
    _currentGenome = _genomesList[bestId];
    _currentSigma = _sigmaList[bestId];
    
    setNewGenomeStatus(true);
    
    // Logging: track descendance
    std::string sLog = std::string("");
    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",descendsFrom," + std::to_string((*_genomesList.begin()).first.first) + "::" + std::to_string((*_genomesList.begin()).first.second) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}


void MovingEEController::performVariation()
{
    TemplateEEController::performVariation();
}

void MovingEEController::broadcastGenome()
{
    TemplateEEController::broadcastGenome();
}

double MovingEEController::getFitness()
{
    return _wm->_fitnessValue;
}

void MovingEEController::resetFitness()
{
    _wm->_fitnessValue = 0;
}


void MovingEEController::updateFitness()
{
    // nothing to do -- updating is performed in AgentObserver (automatic event when energy item are captured)
}

void MovingEEController::logCurrentState()
{
    TemplateEEController::logCurrentState();
}
