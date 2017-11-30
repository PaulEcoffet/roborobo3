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
    
    _NN = nullptr;
    
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
    
    if (_wm->getId() < gNbOfRobots - MovingNSSharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
        _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
}

MovingNSController::~MovingNSController()
{
    _parameters.clear();
    delete _NN;
    _NN = nullptr;
}

void MovingNSController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
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
    
    // Coloring
    if (_isNearObject == false) {
        if (_wm->getId() < gNbOfRobots - MovingNSSharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
            _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
        else // Red LED because we're a fake robot
            _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    }
    
    // Update state variables
    
    _nbNearbyRobots = 0;
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
//                printf("Robot %d (it %d): seeing %d robots on object %d from sensor %d\n", _wm->getId(), gWorld->getIterations(), obj->getNbNearbyRobots(), obj->getId(), i);
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
    
    // how did everyone contribute recently?
    if (MovingNSSharedData::gTotalEffort)
    {
        double avgTotalEffort = 0;
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
    if (_efforts.size() > 0)
    {
        for (auto eff: _efforts)
            avgEffort += eff;
    }
    inputs.push_back(avgEffort);
    
    return inputs;
}

void MovingNSController::stepController()
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
    
    
    // Effort value
    if (MovingNSSharedData::gFixedEffort)
    {
        //Introduce a fixed level of cooperation for all robots so we focus on choosing the optimal number of partners
        _wm->_cooperationLevel = MovingNSSharedData::gFixedEffortValue;
    }
    else
    {
        // Introduce fixed cooperation levels for the last gNbFakeRobots robots
        int nbTrueRobots = gNbOfRobots - MovingNSSharedData::gNbFakeRobots;
        if (_wm->getId() < nbTrueRobots)
            _wm->_cooperationLevel = (outputs[2]+1.0); // in [0, 2]
        else
            _wm->_cooperationLevel = (double)(_wm->getId()-nbTrueRobots)/(double)MovingNSSharedData::gNbFakeRobots * (double)MovingNSSharedData::gFakeCoopValue;
    }
}


void MovingNSController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( _NN != nullptr )
        delete _NN;
    
    switch ( MovingNSSharedData::gControllerType )
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
            std::cerr << "[ERROR] gController type unknown (value: " << MovingNSSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int MovingNSController::computeRequiredNumberOfWeights()
{
    return _NN->getRequiredNumberOfWeights();
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void MovingNSController::performVariation()
{
    if ( MovingNSSharedData::gIndividualMutationRate > random() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
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
    
    for (unsigned int i = 0 ; i < _currentGenome.size() ; i++ )
    {
        double value = _currentGenome[i] + 0 + randgaussian() * _currentSigma;
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
        float randomValue = float(randint()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
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
    
    _nbOutputs = 2+1; // 2 outputs for movement + 1 for cooperation
}

void MovingNSController::initController()
{
    _nbHiddenLayers = MovingNSSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = MovingNSSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();
    
    int nbGenes = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush;
    
    _currentGenome.clear();
    
    // Intialize genomes
    for ( unsigned int i = 0 ; i < nbGenes; i++ )
    {
        _currentGenome.push_back((random()*2.0)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
    // state variables
    _nbNearbyRobots = 0;
    _isNearObject = false;
    _efforts.clear();
    _totalEfforts.clear();
}

void MovingNSController::reset()
{
    initController();
    resetFitness();
}


void MovingNSController::mutateSigmaValue()
{
    float dice = random();
    
    if ( dice <= MovingNSSharedData::gProbaMutation )
    {
        dice = random();
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

void MovingNSController::updatePhenotype()
{
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

// called only once per step (experimentally verified)
void MovingNSController::wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots )
{
//    printf("Robot %d (it %d): near object %d, own effort %lf, total effort %lf, with %d total robots around\n", _wm->getId(), gWorld->getIterations(), __objectId, __effort, __totalEffort, __nbRobots);
    
    if (_wm->getId() < gNbOfRobots - MovingNSSharedData::gNbFakeRobots) // Green LED because we're a true robot (and active)
        _wm->setRobotLED_colorValues(0x32, 0xCD, 0x32);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
    _isNearObject = true;
    _nbNearbyRobots = __nbRobots;
    
    double coeff = MovingNSSharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalEffort, MovingNSSharedData::gConstantA) - __effort;
    

    increaseFitness(payoff);
    _efforts.push_back(__effort);
    if (_efforts.size() >= MovingNSSharedData::gMemorySize)
        _efforts.pop_front();

    _totalEfforts.push_back(__totalEffort);
    if (_totalEfforts.size() >= MovingNSSharedData::gMemorySize)
        _totalEfforts.pop_front();
}

void MovingNSController::dumpGenome()
{
    std::cout <<"Dumping genome of robot #" << _wm->getId() << std::endl;
    std::cout << _currentSigma << " ";
    std::cout << _currentGenome.size() << " ";
    for (auto gene: _currentGenome)
        std::cout << gene << " ";
    std::cout << std::endl;
}
