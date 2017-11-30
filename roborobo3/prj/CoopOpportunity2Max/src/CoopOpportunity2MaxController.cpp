/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "CoopOpportunity2Max/include/CoopOpportunity2MaxController.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxWorldObserver.h"

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

CoopOpportunity2MaxController::CoopOpportunity2MaxController( RobotWorldModel *wm )
{
    _wm = wm;
    
    _NN = nullptr;
    
    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = CoopOpportunity2MaxSharedData::gSigmaRef;
    
    // behaviour
    
    _iteration = 0;

    if ( gEnergyLevel )
        _wm->setEnergyLevel(gEnergyInit);
    
    if ( gNbOfLandmarks > 0 )
        _wm->updateLandmarkSensor(); // wrt closest landmark
    
    reset(); // resetFitness() is called in reset()
    
    if (_wm->getId() < gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
        _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
}

CoopOpportunity2MaxController::~CoopOpportunity2MaxController()
{
    _parameters.clear();
    delete _NN;
    _NN = nullptr;
}

void CoopOpportunity2MaxController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
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
        if (_wm->getId() < gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
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


std::vector<double> CoopOpportunity2MaxController::getInputs()
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
    if (CoopOpportunity2MaxSharedData::gTotalEffort)
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

void CoopOpportunity2MaxController::stepController()
{
    
    // ---- compute and read out ----
    
    _NN->setWeights(_parameters); // set-up NN
    
    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values
    
    _NN->setInputs(inputs);
    
    _NN->step();
    
    std::vector<double> outputs = _NN->readOut();
    
    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;
    
    _wm->_desiredTranslationalValue = outputs[0] * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;
    
    
    // Effort value
    if (CoopOpportunity2MaxSharedData::gFixedEffort)
    {
        //Introduce a fixed level of cooperation for all robots so we focus on choosing the optimal number of partners
        _wm->_cooperationLevel = CoopOpportunity2MaxSharedData::gFixedEffortValue;
    }
    else
    {
        // Introduce fixed cooperation levels for the last gNbFakeRobots robots
        int nbTrueRobots = gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots;
        if (_wm->getId() < nbTrueRobots)
            _wm->_cooperationLevel = (outputs[2]+1.0); // in [0, 2]
        else
            _wm->_cooperationLevel = (double)(_wm->getId()-nbTrueRobots)/(double)CoopOpportunity2MaxSharedData::gNbFakeRobots * (double)CoopOpportunity2MaxSharedData::gFakeCoopValue;
    }
}


void CoopOpportunity2MaxController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    delete _NN;
    
    switch ( CoopOpportunity2MaxSharedData::gControllerType )
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
            std::cerr << "[ERROR] gController type unknown (value: " << CoopOpportunity2MaxSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int CoopOpportunity2MaxController::computeRequiredNumberOfWeights()
{
    return _NN->getRequiredNumberOfWeights();
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void CoopOpportunity2MaxController::performVariation()
{
    if ( CoopOpportunity2MaxSharedData::gIndividualMutationRate > randint() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( CoopOpportunity2MaxSharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(CoopOpportunity2MaxSharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << CoopOpportunity2MaxSharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
}

void CoopOpportunity2MaxController::mutateGaussian(double sigma) // mutate within bounds.
{
    _currentSigma = sigma;
    
    for (double &curWeight : _currentGenome)
    {
        double value = curWeight + 0 + randgaussian() * _currentSigma;
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

        curWeight = value;
    }
}


void CoopOpportunity2MaxController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(randint()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}


void CoopOpportunity2MaxController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs )
    {
        _nbInputs = (1+1+1+1) * _wm->_cameraSensorsNb; // isItAnAgent? + isItAWall? + isItAnObject + nbNearbyRobots
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    
    _nbInputs += 1; // how many robots around?
    
    if (CoopOpportunity2MaxSharedData::gTotalEffort)
        _nbInputs += 1; // what's the total effort given to the object?
    
    _nbInputs += 1; // how much did we contribute?
    
    // wrt outputs
    
    _nbOutputs = 2+1; // 2 outputs for movement + 1 for cooperation
}

void CoopOpportunity2MaxController::initController()
{
    _nbHiddenLayers = CoopOpportunity2MaxSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = CoopOpportunity2MaxSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();
    
    int nbGenes = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush;
    
    _currentGenome.clear();
    
    // Intialize genomes
    for ( unsigned int i = 0 ; i < nbGenes; i++ )
    {
        _currentGenome.push_back((randint()*2.0)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
    // state variables
    _nbNearbyRobots = 0;
    _isNearObject = false;
    _efforts.clear();
    _totalEfforts.clear();
}

void CoopOpportunity2MaxController::reset()
{
    initController();
    resetFitness();
}


void CoopOpportunity2MaxController::mutateSigmaValue()
{
    float dice = randint();
    
    if ( dice <= CoopOpportunity2MaxSharedData::gProbaMutation )
    {
        dice = randint();
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + CoopOpportunity2MaxSharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > CoopOpportunity2MaxSharedData::gSigmaMax)
            {
                _currentSigma = CoopOpportunity2MaxSharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - CoopOpportunity2MaxSharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < CoopOpportunity2MaxSharedData::gSigmaMin )
            {
                _currentSigma = CoopOpportunity2MaxSharedData::gSigmaMin;
            }
        }
    }
}

void CoopOpportunity2MaxController::loadNewGenome( genome __newGenome )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void CoopOpportunity2MaxController::updatePhenotype()
{
    // could be more complicated!
    _parameters = _currentGenome;
}


double CoopOpportunity2MaxController::getFitness()
{
    // nothing to do
    return _wm->_fitnessValue;
}

/*
 * note: resetFitness is first called by the Controller's constructor.
 */
void CoopOpportunity2MaxController::resetFitness()
{
    updateFitness(0);
}

void CoopOpportunity2MaxController::updateFitness( double __newFitness )
{
    if (__newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    _wm->_fitnessValue = __newFitness;
}

void CoopOpportunity2MaxController::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

// called only once per step (experimentally verified)
void CoopOpportunity2MaxController::wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots )
{
//    printf("Robot %d (it %d): near object %d, own effort %lf, total effort %lf, with %d total robots around\n", _wm->getId(), gWorld->getIterations(), __objectId, __effort, __totalEffort, __nbRobots);
    
    if (_wm->getId() < gNbOfRobots - CoopOpportunity2MaxSharedData::gNbFakeRobots) // Green LED because we're a true robot (and active)
        _wm->setRobotLED_colorValues(0x32, 0xCD, 0x32);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
    _isNearObject = true;
    _nbNearbyRobots = __nbRobots;
    
    double coeff = CoopOpportunity2MaxSharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalEffort, CoopOpportunity2MaxSharedData::gConstantA) - __effort;
    
    if (__objectDidMove) {
        increaseFitness(payoff);
        _efforts.push_back(__effort);
        if (_efforts.size() >= CoopOpportunity2MaxSharedData::gMemorySize)
            _efforts.pop_front();
    }
    _totalEfforts.push_back(__totalEffort);
    if (_totalEfforts.size() >= CoopOpportunity2MaxSharedData::gMemorySize)
        _totalEfforts.pop_front();
}

void CoopOpportunity2MaxController::dumpGenome()
{
    std::cout <<"Dumping genome of robot #" << _wm->getId() << std::endl;
    std::cout << _currentSigma << " ";
    std::cout << _currentGenome.size() << " ";
    for (auto gene: _currentGenome)
        std::cout << gene << " ";
    std::cout << std::endl;
}
