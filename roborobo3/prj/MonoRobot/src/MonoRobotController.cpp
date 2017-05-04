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

#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>

using namespace Neural;

MonoRobotController::MonoRobotController( RobotWorldModel *wm )
{
    _wm = wm;
    
    nn = NULL;
    
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
    delete nn;
    nn = NULL;
}

void MonoRobotController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    /*	if (_wm->_id == 0) {
     printf("[DEBUG] dumping robot #0's recent fitness: ");
     double totfit = 0;
     for (auto fit: _fitnesses)
     totfit += fit;
     printf("%lf\n", totfit);
     }
     */
    // If we aren't near an object, write that down (if we were, it's done in the wasNearObject() method)
    if (_isNearObject == false)
        _objectMoves[_iteration%MonoRobotSharedData::gMemorySize] = false;
    
    _movements[_iteration%MonoRobotSharedData::gMemorySize] = fabs(_wm->_actualTranslationalValue); // might be negative
    
    _iteration++;
    
    // * step controller
    
    stepController();
    
    // Update state variables
    
    _isNearObject = false;
    _nbNearbyRobots = 0;
    _fitnesses[_iteration%MonoRobotSharedData::gMemorySize] = 0;
    
}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################


std::vector<double> MonoRobotController::getInputs(){
    
    
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
            //            if ( PhysicalObject::isInstanceOf(objectId) )
            //            {
            //                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
            //                for ( int i = 0 ; i != nbOfTypes ; i++ )
            //                {
            //                    if ( i == gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]->getType() )
            //                        inputs.push_back( 1 ); // match
            //                    else
            //                        inputs.push_back( 0 );
            //                }
            //            }
            //            else
            //            {
            //                // not a physical object. But: should still fill in the inputs (with zeroes)
            //                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
            //                for ( int i = 0 ; i != nbOfTypes ; i++ )
            //                {
            //                    inputs.push_back( 0 );
            //                }
            //            }
            
            // input: another robot? If yes: same group?
            if ( Agent::isInstanceOf(objectId) )
            {
                // this is a robot
                inputs.push_back( 1 );
                
                // same group?
                //                if ( gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->getGroupId() == _wm->getGroupId() )
                //                {
                //                    inputs.push_back( 1 ); // match: same group
                //                }
                //                else
                //                {
                //                    inputs.push_back( 0 ); // not the same group
                //                }
                
                // relative orientation? (ie. angle difference wrt. current agent)
                //                double srcOrientation = _wm->_agentAbsoluteOrientation;
                //                double tgtOrientation = gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->_agentAbsoluteOrientation;
                //                double delta_orientation = - ( srcOrientation - tgtOrientation );
                //                if ( delta_orientation >= 180.0 )
                //                    delta_orientation = - ( 360.0 - delta_orientation );
                //                else
                //                    if ( delta_orientation <= -180.0 )
                //                        delta_orientation = - ( - 360.0 - delta_orientation );
                //                inputs.push_back( delta_orientation/180.0 );
            }
            else
            {
                inputs.push_back( 0 ); // not a robot...
                //               inputs.push_back( 0 ); // ...therefore no match wrt. group.
                //               inputs.push_back( 0 ); // ...and no orientation.
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
    //	if ( _isNearObject )
    //		inputs.push_back(1);
    //	else
    //		inputs.push_back(0);
    
    // how many robots around?
    //	inputs.push_back(_nbNearbyRobots);
    
    // did the object recently move?
//    int nbMoves = 0;
//    for (auto moved: _objectMoves)
//        if (moved)
//            nbMoves++;
//    inputs.push_back(nbMoves);
    
    // how much did we recently move?
//    double totalMovement = 0;
//    for (auto move: _movements)
//        totalMovement += move;
//    inputs.push_back(totalMovement);
    
    // how much fitness did we recently gain?
    double totalFitness = 0;
    for (auto fitness: _fitnesses)
        totalFitness += fitness;
    inputs.push_back(totalFitness);
    
    // are we on the right object?
//    MonoRobotWorldObserver* wobs = static_cast<MonoRobotWorldObserver *>(gWorld->getWorldObserver());
//    int period = MonoRobotSharedData::gNumberOfPeriods*wobs->getGenerationItCount()/MonoRobotSharedData::gEvaluationTime;
//    if ( _isNearObject)
//    {
//        if (_nearbyObjectId == (period+wobs->getStartObjectOffset())%2)
//        {
////            printf("[DEBUG] Right object!\n");
//            inputs.push_back(1);
//        }
//        else
//        {
////            printf("[DEBUG] Wrong object!\n");
//            inputs.push_back(0);
//        }
//    }
//    else
//        inputs.push_back(0);
    
    return inputs;
}

void MonoRobotController::stepController()
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
    
    if ( MonoRobotSharedData::gEnergyRequestOutput )
    {
        _wm->setEnergyRequestValue(outputs[2]);
    }
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
}


void MonoRobotController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( nn != NULL ) // useless: delete will anyway check if nn is NULL or not.
        delete nn;
    
    switch ( MonoRobotSharedData::gControllerType )
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
            std::cerr << "[ERROR] gController type unknown (value: " << MonoRobotSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int MonoRobotController::computeRequiredNumberOfWeights()
{
    unsigned int res = nn->getRequiredNumberOfWeights();
    return res;
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void MonoRobotController::performVariation()
{
    if ( MonoRobotSharedData::gIndividualMutationRate > ranf() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
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


void MonoRobotController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(rand()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}


void MonoRobotController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, can be rewritten to suit your need.
    {
        _nbInputs = ( 1+1 ) * _wm->_cameraSensorsNb; //  ( isItAnAgent?) + isItAWall?
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    if ( gEnergyLevel )
        _nbInputs += 1; // incl. energy level
    if ( gNbOfLandmarks > 0 )
        _nbInputs += 2; // incl. landmark (angle,dist)
    
    //	_nbInputs += 1; // are we near an object?
    
    //	_nbInputs += 1; // how many robots around?
    
    //	_nbInputs += 1; // did the object recently move?
    
    //    _nbInputs += 1; // how much did we recently move?
    
      _nbInputs += 1; // how much fitness did we recently gain?
    
//    _nbInputs += 1; // are we on the object that's giving fitness?
    
    // wrt outputs
    
    _nbOutputs = 2;
    
    if ( MonoRobotSharedData::gEnergyRequestOutput )
        _nbOutputs += 1; // incl. energy request
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
        _currentGenome.push_back((double)(rand()%MonoRobotSharedData::gNeuronWeightRange)/(MonoRobotSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
    // state variables
    _isNearObject = false;
    _nbNearbyRobots = 0;
    for (auto& moved: _objectMoves)
        moved = false;
    for (auto& move: _movements)
        move = 0;
}

void MonoRobotController::reset()
{
    initController();
    resetFitness();
}


void MonoRobotController::mutateSigmaValue()
{
    float dice = float(rand()%100) / 100.0;
    
    if ( dice <= MonoRobotSharedData::gProbaMutation )
    {
        dice = float(rand() %100) / 100.0;
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

void MonoRobotController::loadNewGenome( genome __newGenome )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void MonoRobotController::updatePhenotype() {
    // could be more complicated!
    _parameters = _currentGenome;
}

void MonoRobotController::logCurrentState()
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

double MonoRobotController::getFitness()
{
    // nothing to do
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

void MonoRobotController::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

// called only once per step (experimentally verified)
void MonoRobotController::wasNearObject( int __objectId, bool __objectDidMove, double __gain, int __nbRobots )
{
    //    printf("[DEBUG] Robot %d was near an object at time %d, and could gain %lf fitness\n", _wm->_id, gWorld->getIterations(), __gain);
    _isNearObject = true;
    _nearbyObjectId = __objectId;
    _nbNearbyRobots = __nbRobots;
    if (__objectDidMove || gStuckMovableObjects) {
        // Only give fitness if both robots are on the same object
        if (__nbRobots >= 2)
        {
            //            printf("[DEBUG] fitness!\n");
            increaseFitness(__gain);
            _fitnesses[_iteration%MonoRobotSharedData::gMemorySize] = __gain;
        }
    }
    _objectMoves[_iteration%MonoRobotSharedData::gMemorySize] = __objectDidMove;
}
