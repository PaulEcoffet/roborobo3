/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "MonoRobot/include/MonoRobotController.h"
#include "MonoRobot/include/MonoRobotWorldObserver.h"

#include "World/World.h"
#include "Utilities/Misc.h"
#include <math.h>
#include <string>
#include <algorithm>

#include "World/MovingObject.h"

#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>

using namespace Neural;

MonoRobotController::MonoRobotController( RobotWorldModel *wm )
{
    _wm = wm;
    
    _NN = nullptr;
    
    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = MonoRobotSharedData::gSigmaRef;
    
    // behaviour
    
    _iteration = 0;
    
    _birthdate = 0;
    
    _isListening = true;
    _notListeningDelay = MonoRobotSharedData::gNotListeningStateDelay;
    _listeningDelay = MonoRobotSharedData::gListeningStateDelay;
    
    if ( gEnergyLevel )
        _wm->setEnergyLevel(gEnergyInit);
    
    if ( gNbOfLandmarks > 0 )
        _wm->updateLandmarkSensor(); // wrt closest landmark
    
    reset(); // resetFitness() is called in reset()
    
    _wm->setRobotLED_colorValues(255, 0, 0);
    
}

MonoRobotController::~MonoRobotController()
{
    _parameters.clear();
    delete _NN;
    _NN = nullptr;
}

void MonoRobotController::initController()
{
    _nbHiddenLayers = MonoRobotSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = MonoRobotSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();
    
    unsigned int const nbGene = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush ;
    
    _currentGenome.clear();
    
    // Intialize genome
    
    for ( unsigned int i = 0 ; i != nbGene ; i++ )
    {
        _currentGenome.push_back((double)(randint()%MonoRobotSharedData::gNeuronWeightRange)/(MonoRobotSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
    // state variables
    _isNearObject = false;
    _nbNearbyRobots = 0;
    _efforts.clear();
    _totalEfforts.clear();
}

void MonoRobotController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs )
    {
        _nbInputs = (1+1+1+1) * _wm->_cameraSensorsNb; // isItAnAgent? + isItAWall? + isItAnObject + nbNearbyRobots
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    
    _nbInputs += 1; // how many robots around?
    
    if (MonoRobotSharedData::gTotalEffort)
        _nbInputs += 1; // what's the total effort given to the object?
    
    _nbInputs += 1; // how much did we contribute?
    
    // wrt outputs
    
    _nbOutputs = 2+1; // 2 outputs for movement + 1 for cooperation
}

void MonoRobotController::reset()
{
    initController();
    resetFitness();
}

void MonoRobotController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    
    _iteration++;
    
    // Clean up the memory if we're not on an object
    
    if (_isNearObject == false)
    {
        _efforts.clear();
        _totalEfforts.clear();
    }
    
    // * step controller
    
    stepController();
    
    // Update state variables
    
    _isNearObject = false;
    _nbNearbyRobots = 0;
    
}

std::vector<double> MonoRobotController::getInputs()
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
                int nbDistantRobots = obj->getNbNearbyRobots()+1; //fake robot
//                printf("Robot %d (it %d): seeing %d robots on object %d from sensor %d\n", _wm->getId(), gWorld->getIterations(), nbDistantRobots, obj->getId(), i);
                inputs.push_back(0); // not a robot
                inputs.push_back(0); // not a wall
                inputs.push_back(1); // an object
                inputs.push_back(nbDistantRobots); // some other robots around (+ the fake robot, specific)
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
    
    double avgTotalEffort = 0;
    if (MonoRobotSharedData::gTotalEffort)
    {
        if (_isNearObject == true)
        {
            // the average total effort over the last gMemorySize (at most) turns we were on the object
            for (auto totEff: _totalEfforts)
                avgTotalEffort += totEff;
        }
        inputs.push_back(avgTotalEffort);
    }
    
    // how much did we contribute recently?
    double avgEffort = 0;
    for (auto eff: _efforts)
        avgEffort += eff;
    inputs.push_back(avgEffort);
    
//    printf("Robot %d (it %d) NN: nbNearby %d, avgTotalEffort %lf, avgEffort %lf, isNearObject: %s\n", _wm->getId(), gWorld->getIterations(), _nbNearbyRobots, avgTotalEffort, avgEffort, _isNearObject?"yes":"no");
    
    return inputs;
}

void MonoRobotController::stepController()
{
    
    // ---- compute and read out ----
    
    _NN->setWeights(_parameters); // set-up NN
    
    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values
    
    _NN->setInputs(inputs);
    
    _NN->step();
    
    std::vector<double> outputs = _NN->readOut();
    
    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;
    
    _wm->_desiredTranslationalValue = outputs[0];
    _wm->_desiredRotationalVelocity = outputs[1];
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
    _wm->_cooperationLevel = outputs[2] + 1.0; // in [0, 2]
}


void MonoRobotController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( _NN != nullptr ) // useless: delete will anyway check if nn is NULL or not.
        delete _NN;
    
    switch ( MonoRobotSharedData::gControllerType )
    {
        case 0:
        {
            // MLP
            _NN = new MLP(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        case 1:
        {
            // PERCEPTRON
            _NN = new Perceptron(_parameters, _nbInputs, _nbOutputs);
            break;
        }
        case 2:
        {
            // ELMAN
            _NN = new Elman(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        default: // default: no controller
            std::cerr << "[ERROR] gController type unknown (value: " << MonoRobotSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int MonoRobotController::computeRequiredNumberOfWeights()
{
    return _NN->getRequiredNumberOfWeights();
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void MonoRobotController::performVariation()
{
    if ( MonoRobotSharedData::gIndividualMutationRate > random() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( MonoRobotSharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(MonoRobotSharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << MonoRobotSharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
}

void MonoRobotController::mutateGaussian(float sigma) // mutate within bounds.
{
    _currentSigma = sigma;
    
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        double value = _currentGenome[i] + randgaussian() * _currentSigma;
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


void MonoRobotController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = random(); // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}

void MonoRobotController::mutateSigmaValue()
{
    float dice = random();
    
    if ( dice <= MonoRobotSharedData::gProbaMutation )
    {
        dice = random();
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + MonoRobotSharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > MonoRobotSharedData::gSigmaMax)
            {
                _currentSigma = MonoRobotSharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - MonoRobotSharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < MonoRobotSharedData::gSigmaMin )
            {
                _currentSigma = MonoRobotSharedData::gSigmaMin;
            }
        }
    }
}

void MonoRobotController::loadNewGenome( genome __newGenome, bool __mutate )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    if (__mutate)
    {
        performVariation();
    }
    updatePhenotype();
}

void MonoRobotController::updatePhenotype() {
    // could be more complicated!
    _parameters = _currentGenome;
}

void MonoRobotController::logCurrentState()
{
    // Logging
    std::string sLog = "" + std::to_string(gWorld->getIterations()) +
    "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) +
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

double MonoRobotController::getFitness()
{
    return _wm->_fitnessValue;
}

/*
 * note: resetFitness is first called by the Controller's constructor.
 */
void MonoRobotController::resetFitness()
{
    updateFitness(0);
}

void MonoRobotController::updateFitness( double __newFitness )
{
    if (__newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    _wm->_fitnessValue = __newFitness;
}


// Note: fitnesses can decrease when we make a bad deal!
// Just ensure they remain positive.
void MonoRobotController::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

// called only once per step (experimentally verified)
void MonoRobotController::wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots )
{
    
    // Experiment-specific: 1 fake robot that contributes 0.25*(objectID%8)
    __nbRobots += 1;
    __totalEffort += 0.25*((double)(__objectId%8));
    
//    printf("[DEBUG] Robot %d (it %d) was near object %d, own effort %lf, total effort %lf, with %d total robots around\n", _wm->getId(), gWorld->getIterations(), __objectId, __effort, __totalEffort, __nbRobots);
    
    _isNearObject = true;
    _nearbyObjectId = __objectId;
    _nbNearbyRobots = __nbRobots;
    
    double coeff = MonoRobotSharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalEffort, MonoRobotSharedData::gConstantA) - __effort;

//       printf("[DEBUG] Robot %d (it %d): effort %lf, payoff %lf\n", _wm->getId(), gWorld->getIterations(), __effort, payoff);
    increaseFitness(payoff);
    _efforts.push_back(__effort);
    if (_efforts.size() >= MonoRobotSharedData::gMemorySize)
        _efforts.pop_front();
    _totalEfforts.push_back(__totalEffort);
    if (_totalEfforts.size() >= MonoRobotSharedData::gMemorySize)
        _totalEfforts.pop_front();

}


void MonoRobotController::dumpGenome()
{
    printf("Robot %d it %d: ", _wm->getId(), gWorld->getIterations());
    printf("%lf,", _currentSigma);
    for (auto v: _currentGenome)
        printf("%lf ", v);
    printf("\n");
}
