/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "SingleGenome/include/SingleGenomeController.h"
#include "SingleGenome/include/SingleGenomeWorldObserver.h"

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

SingleGenomeController::SingleGenomeController( RobotWorldModel *wm )
{
    _wm = wm;
    
    _NN = nullptr;
    
    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = SingleGenomeSharedData::gSigmaRef;
    
    // behaviour
    
    _iteration = 0;
    
    _birthdate = 0;
    
    _isListening = true;
    _notListeningDelay = SingleGenomeSharedData::gNotListeningStateDelay;
    _listeningDelay = SingleGenomeSharedData::gListeningStateDelay;
        
    if ( gEnergyLevel )
        _wm->setEnergyLevel(gEnergyInit);
    
    if ( gNbOfLandmarks > 0 )
        _wm->updateLandmarkSensor(); // wrt closest landmark
    
    reset(); // resetFitness() is called in reset()
    
    _wm->setRobotLED_colorValues(255, 0, 0);
    
}

SingleGenomeController::~SingleGenomeController()
{
    _parameters.clear();
    delete _NN;
    _NN = nullptr;
}

void SingleGenomeController::initController()
{
    _nbHiddenLayers = SingleGenomeSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = SingleGenomeSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();
    
    if ( gVerbose )
        std::cout << std::flush;
    
    _currentGenome.clear();
    
    // We'll get our genome from the WorldObserver
    
    // state variables
    _nbNearbyRobots = 0;
    for (auto& eff: _efforts)
        eff = 0;
    for (auto& totEff: _totalEfforts)
        totEff = 0;
    _objectTime = 0;
}

void SingleGenomeController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, can be rewritten to suit your need.
    {
        _nbInputs = (1+1+1+1) * _wm->_cameraSensorsNb; // isItAnAgent? + isItAWall? + isItAnObject + nbNearbyRobots
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    
    _nbInputs += 1; // how many robots around?
    
    if (SingleGenomeSharedData::gTotalEffort)
        _nbInputs += 1; // what's the total effort given to the object?
    
    _nbInputs += 1; // how much did we contribute?
    
    // wrt outputs
    
    _nbOutputs = 2+1; // 2 outputs for movement + 1 for cooperation
}

void SingleGenomeController::reset()
{
    initController();
    resetFitness();
}

void SingleGenomeController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    
    _iteration++;
    
    // * step controller

    stepController();
    
    // Coloring
    
    if (_isNearObject == false) // blue
        _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
    
    // Logging
    
    Robot* robot = gWorld->getRobot(_wm->getId());
    SingleGenomeAgentObserver *aobs = static_cast<SingleGenomeAgentObserver *>(robot->getObserver());
    aobs->logStats();

    // Update state variables
    
	_nbNearbyRobots = 0;
    _efforts[_iteration%SingleGenomeSharedData::gMemorySize] = 0;
    _totalEfforts[_iteration%SingleGenomeSharedData::gMemorySize] = 0;
    _isNearObject = false;

}


std::vector<double> SingleGenomeController::getInputs()
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
                inputs.push_back(obj->getNbNearbyRobots()+SingleGenomeSharedData::gFakeRobotsPerObject); // some other robots around
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

    if (SingleGenomeSharedData::gTotalEffort)
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

void SingleGenomeController::stepController()
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
    
    if ( SingleGenomeSharedData::gEnergyRequestOutput )
    {
        _wm->setEnergyRequestValue(outputs[2]);
    }
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
    _wm->_cooperationLevel = outputs[2] + 1.0; // in [0, 2]
    
}


void SingleGenomeController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( _NN != NULL ) // useless: delete will anyway check if nn is NULL or not.
        delete _NN;
    
    switch ( SingleGenomeSharedData::gControllerType )
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
            std::cerr << "[ERROR] gController type unknown (value: " << SingleGenomeSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int SingleGenomeController::computeRequiredNumberOfWeights()
{
    unsigned int res = _NN->getRequiredNumberOfWeights();
    return res;
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void SingleGenomeController::performVariation()
{
    if ( SingleGenomeSharedData::gIndividualMutationRate > ranf() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( SingleGenomeSharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(SingleGenomeSharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << SingleGenomeSharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
}

void SingleGenomeController::mutateGaussian(float sigma) // mutate within bounds.
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


void SingleGenomeController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(rand()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}

void SingleGenomeController::mutateSigmaValue()
{
    float dice = ranf();
    
    if ( dice <= SingleGenomeSharedData::gProbaMutation )
    {
        dice = ranf();
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + SingleGenomeSharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > SingleGenomeSharedData::gSigmaMax)
            {
                _currentSigma = SingleGenomeSharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - SingleGenomeSharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < SingleGenomeSharedData::gSigmaMin )
            {
                _currentSigma = SingleGenomeSharedData::gSigmaMin;
            }
        }
    }
}

void SingleGenomeController::loadNewGenome( genome __newGenome )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void SingleGenomeController::updatePhenotype() {
    // could be more complicated!
    _parameters = _currentGenome;
}

void SingleGenomeController::logCurrentState()
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

double SingleGenomeController::getFitness()
{
    // nothing to do
    return _wm->_fitnessValue;
}

void SingleGenomeController::resetFitness()
{
    updateFitness(0);
}

void SingleGenomeController::updateFitness( double __newFitness )
{
	if (__newFitness < 0)
	{
		updateFitness(0);
		return;
	}
    _wm->_fitnessValue = __newFitness;
}

void SingleGenomeController::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

// called only once per step (experimentally verified)
void SingleGenomeController::wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots )
{
    if (__effort > 0) // Green LED
        _wm->setRobotLED_colorValues(0x32, 0xCD, 0x32);
    else // Red LED
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
    // Fake object / effort setup
    SingleGenomeWorldObserver *wobs = static_cast<SingleGenomeWorldObserver *>(gWorld->getWorldObserver());
    __nbRobots += wobs->getNbFakeRobots();
    __totalEffort += wobs->getNbFakeRobots()*wobs->getFakeCoop();
    
//    printf("[DEBUG] Robot %d was near object %d, own effort %lf, total effort %lf, with %d total robots around\n", _wm->getId(), __objectId, __effort, __totalEffort, __nbRobots);
    
    _isNearObject = true;
    _nbNearbyRobots = __nbRobots;
    _objectTime++;
    
    double coeff = SingleGenomeSharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalEffort, SingleGenomeSharedData::gConstantA) - __effort;
    
    if (__objectDidMove || gStuckMovableObjects) {
//        printf("[DEBUG] Robot %d (it %d): effort %lf, payoff %lf\n", _wm->getId(), gWorld->getIterations()%1000, __effort, payoff);
        increaseFitness(payoff);
        _efforts[_iteration%SingleGenomeSharedData::gMemorySize] = __effort;
        _totalEfforts[_iteration%SingleGenomeSharedData::gMemorySize] = __totalEffort;
    }

}

void SingleGenomeController::dumpGenome()
{
    std::cout <<"Dumping genome of robot #" << _wm->getId() << std::endl;
    std::cout << _currentSigma << " ";
    std::cout << _currentGenome.size() << " ";
    for (auto gene: _currentGenome)
        std::cout << gene << " ";
    std::cout << std::endl;
}
