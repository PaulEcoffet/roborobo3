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

#include "World/MovingObject.h"

using namespace Neural;

MovingNSController::MovingNSController( RobotWorldModel *wm )
{
    _wm = wm;
    
    _movementNN = nullptr;
    _coopNN = nullptr;
    
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
    
    reset(); // resetFitness() is called in reset()
    
    _wm->setRobotLED_colorValues(255, 0, 0);
    
}

MovingNSController::~MovingNSController()
{
    for (auto& params: _parameters)
        params.clear();
    delete _movementNN;
    _movementNN = NULL;
}

void MovingNSController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    _iteration++;
    
    // * step controller

    stepController();
    
    // Coloring
    
    if (_isNearObject == false) // blue
        _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);

    // Update state variables
    
	_nbNearbyRobots = 0;
    _efforts[_iteration%MovingNSSharedData::gMemorySize] = 0;
    _totalEfforts[_iteration%MovingNSSharedData::gMemorySize] = 0;
    _isNearObject = false;
    
}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################


std::vector<double> MovingNSController::getInputs()
{
    std::vector<double> inputs;
    
    
    // distance sensors
    for(int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        inputs.push_back( _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i) );
        
        if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, should be rewritten to suit your need.
        {
            int entityId = _wm->getObjectIdFromCameraSensor(i);
            
            if (Agent::isInstanceOf(entityId)) // it's a robot
            {
                inputs.push_back(1); // a robot
                inputs.push_back(0); // not a wall
                inputs.push_back(0); // not an object
                inputs.push_back(0); // no other robots around
            }
            else if (entityId == 0) // it's a wall
            {
                inputs.push_back(0); // not a robot
                inputs.push_back(1); // a wall
                inputs.push_back(0); // not an object
                inputs.push_back(0); // no robots around
            }
            else if (entityId >= gPhysicalObjectIndexStartOffset) // an object
            {
                MovingObject* obj = static_cast<MovingObject *>(gPhysicalObjects[entityId-gPhysicalObjectIndexStartOffset]);
                inputs.push_back(0); // not a robot
                inputs.push_back(0); // not a wall
                inputs.push_back(1); // an object
                inputs.push_back(obj->getNbNearbyRobots()); // some other robots around
            }
            else // found nothing
            {
                inputs.push_back(0); // not a robot
                inputs.push_back(0); // not a wall
                inputs.push_back(0); // not an object
                inputs.push_back(0); // no robots around
            }
            
        }
    }
    
    
    // floor sensor
    inputs.push_back( (double)_wm->getGroundSensor_redValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_greenValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_blueValue()/255.0 );

	// how many robots around?
	inputs.push_back(_nbNearbyRobots);

    if (MovingNSSharedData::gTotalEffort)
    {
        // what's the total effort given to the object in the last few turns?
        double totalEffort = 0;
        for (auto eff: _totalEfforts)
            totalEffort += eff;
        inputs.push_back(totalEffort);
    }
    
    // how much did we contribute?
    double effort = 0;
    for (auto eff: _efforts)
        effort += eff;
    inputs.push_back(effort);

    return inputs;
}

void MovingNSController::stepController()
{

    // ---- compute and read out ----
    
    _movementNN->setWeights(_parameters[0]); // set-up NN
    _coopNN->setWeights(_parameters[1]);
    
    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values
    
    _movementNN->setInputs(inputs);
    _coopNN->setInputs(inputs);
    
    _movementNN->step();
    _coopNN->step();
    
    std::vector<double> movementOutputs = _movementNN->readOut();
    std::vector<double> coopOutputs = _coopNN->readOut();
    
    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;
    
    _wm->_desiredTranslationalValue = movementOutputs[0];
    _wm->_desiredRotationalVelocity = movementOutputs[1];
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
    _wm->_cooperationLevel = (coopOutputs[0]+1.0)/2.0;
    
}


void MovingNSController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( _movementNN != nullptr )
        delete _movementNN;
    
    if (_coopNN != nullptr)
        delete _coopNN;
    
    switch ( MovingNSSharedData::gControllerType )
    {
        case 0:
        {
            // MLP
            _movementNN = new MLP(_parameters[0], _nbInputs, _nbOutputs[0], *(_nbNeuronsPerHiddenLayer));
            _coopNN = new MLP(_parameters[1], _nbInputs, _nbOutputs[1], *(_nbNeuronsPerHiddenLayer));
            break;
        }
        case 1:
        {
            // PERCEPTRON
            _movementNN = new Perceptron(_parameters[0], _nbInputs, _nbOutputs[0]);
            _coopNN = new Perceptron(_parameters[1], _nbInputs, _nbOutputs[1]);

            break;
        }
        case 2:
        {
            // ELMAN
            _movementNN = new Elman(_parameters[0], _nbInputs, _nbOutputs[0], *(_nbNeuronsPerHiddenLayer));
            _coopNN = new Elman(_parameters[1], _nbInputs, _nbOutputs[1], *(_nbNeuronsPerHiddenLayer));
            break;
        }
        default: // default: no controller
            std::cerr << "[ERROR] gController type unknown (value: " << MovingNSSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int MovingNSController::computeRequiredNumberOfWeights(int __NN)
{
    if (__NN == 0)
        return _movementNN->getRequiredNumberOfWeights();
    else
        return _coopNN->getRequiredNumberOfWeights();
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void MovingNSController::performVariation()
{
    if ( MovingNSSharedData::gIndividualMutationRate > ranf() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
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
    
    for (int iNN = 0; iNN < 2; iNN++) {
        for (unsigned int i = 0 ; i != _currentGenome[iNN].size() ; i++ )
        {
            double value = _currentGenome[iNN][i] + getGaussianRand(0,_currentSigma);
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
            
            _currentGenome[iNN][i] = value;
        }
    }
}


void MovingNSController::mutateUniform() // mutate within bounds.
{
    for (int iNN = 0; iNN < 2; iNN++)
    {
        for (unsigned int i = 0 ; i != _currentGenome[iNN].size() ; i++ )
        {
            float randomValue = float(rand()%100) / 100.0; // in [0,1[
            double range = _maxValue - _minValue;
            double value = randomValue * range + _minValue;
            
            _currentGenome[iNN][i] = value;
        }
    }
}


void MovingNSController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs )
    {
        _nbInputs = (1+1+1+1) * _wm->_cameraSensorsNb; // isItAnAgent? + isItAWall? + isItAnObject + nbNearbyRobots
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    
    _nbInputs += 1; // how many robots around?
    
    if (MovingNSSharedData::gTotalEffort)
        _nbInputs += 1; // what's the total effort given to the object?
    
    _nbInputs += 1; // how much did we contribute?
    
    // wrt outputs
    
    _nbOutputs[0] = 2; // 2 outputs for movement
    _nbOutputs[1] = 1; // 1 output for cooperation
}

void MovingNSController::initController()
{
    _nbHiddenLayers = MovingNSSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = MovingNSSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();

    int nbGene[2];
    for (int iNN = 0; iNN < 2; iNN++)
        nbGene[iNN] = computeRequiredNumberOfWeights(iNN);
    
    if ( gVerbose )
        std::cout << std::flush ;
    
    for (auto& gen: _currentGenome)
        gen.clear();
    
    // Intialize genomes
    for (int iNN = 0; iNN < 2; iNN++) {
        for ( unsigned int i = 0 ; i != nbGene[iNN] ; i++ )
        {
            _currentGenome[iNN].push_back((double)(rand()%MovingNSSharedData::gNeuronWeightRange)/(MovingNSSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
        }
    }
    
    updatePhenotype();

	// state variables
	_nbNearbyRobots = 0;
    for (auto& eff: _efforts)
        eff = 0;
    for (auto& totEff: _totalEfforts)
        totEff = 0;
    _coopTime = 0;
    _objectTime = 0;
}

void MovingNSController::reset()
{
    initController();
    resetFitness();
}


void MovingNSController::mutateSigmaValue()
{
    float dice = ranf();
    
    if ( dice <= MovingNSSharedData::gProbaMutation )
    {
        dice = ranf();
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
    for (int iNN = 0; iNN < 2; iNN++)
        _currentGenome[iNN] = __newGenome.first[iNN];
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void MovingNSController::updatePhenotype() {

    for (int iNN = 0; iNN < 2; iNN++)
        _parameters[iNN] = _currentGenome[iNN];
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

// called only once per step (experimentally verified)
void MovingNSController::wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots )
{
    printf("[DEBUG] Robot %d was near object %d, own cooperation %lf, total cooperation %lf, with %d total robots around\n", _wm->getId(), __objectId, __effort, __totalEffort, __nbRobots);
    
    if (__effort > 0) // Green LED
        _wm->setRobotLED_colorValues(0x32, 0xCD, 0x32);
    else // Red LED
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
    _isNearObject = true;
    _objectTime++;
    if (__effort > 0)
        _coopTime++;
    
    double coeff = MovingNSSharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalEffort, MovingNSSharedData::gConstantA) - __effort;
    
    if (__objectDidMove || gStuckMovableObjects) {
//        printf("[DEBUG] Robot %d (it %d): effort %lf, payoff %lf\n", _wm->getId(), gWorld->getIterations()%1000, __effort, payoff);
        increaseFitness(payoff);
        _efforts[_iteration%MovingNSSharedData::gMemorySize] = __effort;
        _totalEfforts[_iteration%MovingNSSharedData::gMemorySize] = __totalEffort;
    }

}

void MovingNSController::dumpGenome()
{
    std::cout <<"Dumping genome of robot #" << _wm->getId() << std::endl;
    std::cout << _currentSigma << " ";
    for (int iNN = 0; iNN < 2; iNN++)
    {
        std::cout << _currentGenome[iNN].size() << " ";
        for (auto gene: _currentGenome[iNN])
            std::cout << gene << " ";
    }
    std::cout << std::endl;
}
