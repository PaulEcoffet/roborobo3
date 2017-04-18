/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "MovingNS/include/MovingNSController.h"
#include "MovingNS/include/MovingNSWorldObserver.h"

#include "World/World.h"
#include "Utilities/Misc.h"
#include <math.h>
#include <string>
#include <algorithm>

#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>

using namespace Neural;

MovingNSController::MovingNSController( RobotWorldModel *wm )
{
    _wm = wm;
    
    nn = NULL;
    
    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = MovingNSSharedData::gSigmaRef;
    
    // behaviour
    
    _iteration = 0;
    
    _birthdate = 0;
    
    _isListening = true;
    _notListeningDelay = MovingNSSharedData::gNotListeningStateDelay;
    _listeningDelay = MovingNSSharedData::gListeningStateDelay;
        
    if ( gEnergyLevel )
        _wm->setEnergyLevel(gEnergyInit);
    
    if ( gNbOfLandmarks > 0 )
        _wm->updateLandmarkSensor(); // wrt closest landmark
    
    reset();
    resetFitness();
    
    _wm->setRobotLED_colorValues(255, 0, 0);
    
}

MovingNSController::~MovingNSController()
{
    _parameters.clear();
    delete nn;
    nn = NULL;
}

void MovingNSController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    _iteration++;
    
    // * step controller

    stepController();
    
    // Update state variables
    
    _isNearObject = false;
    
}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################


std::vector<double> MovingNSController::getInputs(){
    
    
    // WHAT FOLLOWS IS AN EXAMPLE OF LOADING A NN-CONTROLER WITH A FULL-FLEDGE SENSORY INPUT INFORMATION
    // Rewrite to match your own extended input scheme, if needed.
    // Note that you may tune it on/off using gExtendedSensoryInputs defined in the properties file.
    // When modifying this code, dont forget to update the initialization in the reset() method
    // Example:
    //      - you may want to distinguish between robot's groups (if more than 1)
    //      - you may want to restrict the number of objects that can be identified (if not all possible objects are in use)
    //      - you may want to compress inputs (e.g. binary encoding instead of one-input-per-object-type.
    //      - (...)
    //
    // In the following, sensory inputs for each sensor are ordered (and filled in) as follow:
    // - N range sensors
    //      - distance to obstacle (max if nothing)
    //      IF extendedInputs is True:
    //          - [0...N_physicalobjecttypes] Is it an object of type i? (0: no, 1: yes) -- type: 0 (round), 1 (energy item), 2 (gate), 3 (switch), ...? (cf. PhysicalObjectFactory)
    //          - is it a robot? (1 or 0)
    //          - is it from the same group? (1 or 0)
    //          - relative orientation wrt. current robot (relative orientation or 0 if not a robot)
    //          - is it a wall? (ie. a non-distinguishible something) (1 or 0)
    // - floor sensor, red value
    // - floor sensor, green value
    // - floor sensor, blue value
    // - relative direction to the closest landmark
    // - distance to the closest landmark
    // - normalized energy level (if any)
    //
    // => number of sensory inputs: N * rangeSensors + 6, with rangeSensors varying from 1 to many, if extended sensory inputs are on.
    

    
    std::vector<double> inputs;
    
    // distance sensors
    for(int i  = 0; i < _wm->_cameraSensorsNb; i++)
    {
        inputs.push_back( _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i) );
        
        if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, should be rewritten to suit your need.
        {
            int objectId = _wm->getObjectIdFromCameraSensor(i);
            
            // input: physical object? which type?
            if ( PhysicalObject::isInstanceOf(objectId) )
            {
                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
                for ( int i = 0 ; i != nbOfTypes ; i++ )
                {
                    if ( i == gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]->getType() )
                        inputs.push_back( 1 ); // match
                    else
                        inputs.push_back( 0 );
                }
            }
            else
            {
                // not a physical object. But: should still fill in the inputs (with zeroes)
                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
                for ( int i = 0 ; i != nbOfTypes ; i++ )
                {
                    inputs.push_back( 0 );
                }
            }
            
            // input: another agent? If yes: same group?
            if ( Agent::isInstanceOf(objectId) )
            {
                // this is an agent
                inputs.push_back( 1 );
                
                // same group?
                if ( gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->getGroupId() == _wm->getGroupId() )
                {
                    inputs.push_back( 1 ); // match: same group
                }
                else
                {
                    inputs.push_back( 0 ); // not the same group
                }
                
                // relative orientation? (ie. angle difference wrt. current agent)
                double srcOrientation = _wm->_agentAbsoluteOrientation;
                double tgtOrientation = gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->_agentAbsoluteOrientation;
                double delta_orientation = - ( srcOrientation - tgtOrientation );
                if ( delta_orientation >= 180.0 )
                    delta_orientation = - ( 360.0 - delta_orientation );
                else
                    if ( delta_orientation <= -180.0 )
                        delta_orientation = - ( - 360.0 - delta_orientation );
                inputs.push_back( delta_orientation/180.0 );
            }
            else
            {
                inputs.push_back( 0 ); // not an agent...
                inputs.push_back( 0 ); // ...therefore no match wrt. group.
                inputs.push_back( 0 ); // ...and no orientation.
            }
            
            // input: wall or empty?
            if ( objectId >= 0 && objectId < gPhysicalObjectIndexStartOffset ) // not empty, but cannot be identified: this is a wall.
                inputs.push_back( 1 );
            else
                inputs.push_back( 0 ); // nothing. (objectId=-1)
        }
    }
    
    
    // floor sensor
    inputs.push_back( (double)_wm->getGroundSensor_redValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_greenValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_blueValue()/255.0 );
    
    // landmark (closest landmark)
    if ( gNbOfLandmarks > 0 )
    {
        _wm->updateLandmarkSensor(); // update with closest landmark
        inputs.push_back( _wm->getLandmarkDirectionAngleValue() );
        inputs.push_back( _wm->getLandmarkDistanceValue() );
    }
    
    
    // energy level
    if ( gEnergyLevel )
    {
        inputs.push_back( _wm->getEnergyLevel() / gEnergyMax );
    }
    
	// are we near an object?
	if ( _isNearObject )
		inputs.push_back(1);
	else
		inputs.push_back(0);

	// how many robots around?
	inputs.push_back(_nbNearbyRobots);
    
    return inputs;
}

void MovingNSController::stepController()
{

    // ---- compute and read out ----
    
    nn->setWeights(_parameters); // set-up NN
    
    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values
    
    nn->setInputs(inputs);
    
    nn->step();
    
    std::vector<double> outputs = nn->readOut();
    
    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;
    
    _wm->_desiredTranslationalValue = outputs[0];
    _wm->_desiredRotationalVelocity = outputs[1];
    
    if ( MovingNSSharedData::gEnergyRequestOutput )
    {
        _wm->setEnergyRequestValue(outputs[2]);
    }
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
}


void MovingNSController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( nn != NULL ) // useless: delete will anyway check if nn is NULL or not.
        delete nn;
    
    switch ( MovingNSSharedData::gControllerType )
    {
        case 0:
        {
            // MLP
            nn = new MLP(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        case 1:
        {
            // PERCEPTRON
            nn = new Perceptron(_parameters, _nbInputs, _nbOutputs);
            break;
        }
        case 2:
        {
            // ELMAN
            nn = new Elman(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        default: // default: no controller
            std::cerr << "[ERROR] gController type unknown (value: " << MovingNSSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int MovingNSController::computeRequiredNumberOfWeights()
{
    unsigned int res = nn->getRequiredNumberOfWeights();
    return res;
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void MovingNSController::performVariation()
{
    if ( MovingNSSharedData::gIndividualMutationRate > rand()/RAND_MAX ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( MovingNSSharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(MovingNSSharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << MovingNSSharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
}

void MovingNSController::mutateGaussian(float sigma) // mutate within bounds.
{
    _currentSigma = sigma;
    
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        double value = _currentGenome[i] + getGaussianRand(0,_currentSigma);
        // bouncing upper/lower bounds
        if ( value < _minValue )
        {
            double range = _maxValue - _minValue;
            double overflow = - ( (double)value - _minValue );
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _minValue + overflow;
            else // overflow btw range and range*2
                value = _minValue + range - (overflow-range);
        }
        else if ( value > _maxValue )
        {
            double range = _maxValue - _minValue;
            double overflow = (double)value - _maxValue;
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _maxValue - overflow;
            else // overflow btw range and range*2
                value = _maxValue - range + (overflow-range);
        }
        
        _currentGenome[i] = value;
    }
    
}


void MovingNSController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(rand()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}


void MovingNSController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, can be rewritten to suit your need.
    {
        _nbInputs = ( PhysicalObjectFactory::getNbOfTypes()+3+1 ) * _wm->_cameraSensorsNb; // nbOfTypes + ( isItAnAgent? + isItSameGroupId? + agentAngleDifference?) + isItAWall?
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    if ( gEnergyLevel )
        _nbInputs += 1; // incl. energy level
    if ( gNbOfLandmarks > 0 )
        _nbInputs += 2; // incl. landmark (angle,dist)
    
	_nbInputs += 1; // near an object?
	
	_nbInputs += 1; // how many robots around?

    // wrt outputs
    
    _nbOutputs = 2;
    
    if ( MovingNSSharedData::gEnergyRequestOutput )
        _nbOutputs += 1; // incl. energy request
}

void MovingNSController::initController()
{
    _nbHiddenLayers = MovingNSSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = MovingNSSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();

    unsigned int const nbGene = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush ;
    
    _currentGenome.clear();
    
    // Intialize genome
    
    for ( unsigned int i = 0 ; i != nbGene ; i++ )
    {
        _currentGenome.push_back((double)(rand()%MovingNSSharedData::gNeuronWeightRange)/(MovingNSSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
}

void MovingNSController::reset()
{
    initController();
    resetFitness();
}


void MovingNSController::mutateSigmaValue()
{
    float dice = float(rand()%100) / 100.0;
    
    if ( dice <= MovingNSSharedData::gProbaMutation )
    {
        dice = float(rand() %100) / 100.0;
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + MovingNSSharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > MovingNSSharedData::gSigmaMax)
            {
                _currentSigma = MovingNSSharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - MovingNSSharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < MovingNSSharedData::gSigmaMin )
            {
                _currentSigma = MovingNSSharedData::gSigmaMin;
            }
        }
    }
}

void MovingNSController::loadNewGenome( genome __newGenome )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void MovingNSController::updatePhenotype() {
    // could be more complicated!
    _parameters = _currentGenome;
}

void MovingNSController::logCurrentState()
{
    // Logging
    std::string sLog = "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) +
    ",age," + std::to_string(gWorld->getIterations()-_birthdate) +
    ",energy," +  std::to_string(_wm->getEnergyLevel()) +
    ",sigma," + std::to_string(_currentSigma) +
    ",x_init," + std::to_string(_wm->getXReal()) +
    ",y_init," + std::to_string(_wm->getYReal()) +
    ",x_current," + std::to_string(_Xinit) +
    ",y_current," + std::to_string(_Yinit) +
    ",dist," + std::to_string( getEuclideanDistance( _Xinit, _Yinit, _wm->getXReal(), _wm->getYReal() ) ) +
    ",sumOfDist," + std::to_string( _dSumTravelled ) +
    ",groupId," + std::to_string(_wm->getGroupId()) +
    ",fitnessValue," + std::to_string(_wm->_fitnessValue) + 
    "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}

double MovingNSController::getFitness()
{
    // nothing to do
    return _wm->_fitnessValue;
}

/*
 * note: resetFitness is first called by the Controller's constructor.
 */
void MovingNSController::resetFitness()
{
    updateFitness(0);
}

void MovingNSController::updateFitness( double __newFitness )
{
	if (__newFitness < 0)
	{
		updateFitness(0);
		return;
	}
    _wm->_fitnessValue = __newFitness;
}

void MovingNSController::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

void MovingNSController::wasNearObject( bool __objectDidMove, double __gain, int __nbRobots )
{
    _isNearObject = true;
	_nbNearbyRobots = __nbRobots;
    if (__objectDidMove)
        increaseFitness(__gain);
}
