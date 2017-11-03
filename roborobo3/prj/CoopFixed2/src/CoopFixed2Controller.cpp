/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */

#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"

#include "Utilities/Misc.h"

#include "neuralnetworks/MLP.h"
#include "neuralnetworks/Perceptron.h"
#include "neuralnetworks/Elman.h"
#include "CoopFixed2/include/CoopFixed2OpportunityObj.h"


using namespace Neural;

CoopFixed2Controller::CoopFixed2Controller( RobotWorldModel *wm )
{
    _wm = wm;
    
    _NN = nullptr;
    
    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = CoopFixed2SharedData::gSigmaRef;
    
    // behaviour

    _canMove = true;

    
    reset(); // resetFitness() is called in reset()
    
    if (_wm->getId() < gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
        _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
}

CoopFixed2Controller::~CoopFixed2Controller()
{
    _parameters.clear();
    delete _NN;
    _NN = nullptr;
}

void CoopFixed2Controller::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    // Clean up the memory if we're not on an object
    
    if (!_isNearObject)
    {
        _efforts.clear();
        _totalEfforts.clear();
    }
    
    // * step controller
    
    stepController();
    
    // Coloring
    if (!_isNearObject) {
        if (_wm->getId() < gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots) // Blue LED because we're a true robot (and inactive)
            _wm->setRobotLED_colorValues(0x00, 0x99, 0xFF);
        else // Red LED because we're a fake robot
            _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    }

    if (! _canMove)
    {
        _wm->setRobotLED_colorValues(0xFF, 0xDD, 0);
    }
    
    // Update state variables
    
    _nbNearbyRobots = 0;
    _isNearObject = false;
    _canMove = true;

}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################


std::vector<double> CoopFixed2Controller::getInputs()
{
    std::vector<double> inputs;
    inputs.reserve(_nbInputs); // Allocate all the needed memory
    
    
    // distance sensors
    for(int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        inputs.push_back( _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i) );
        
        if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, should be rewritten to suit your need.
        {
            auto entityId = (int) _wm->getObjectIdFromCameraSensor(i);
            
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
                auto* obj = dynamic_cast<CoopFixed2OpportunityObj *>(gPhysicalObjects[entityId-gPhysicalObjectIndexStartOffset]);
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
    if (CoopFixed2SharedData::gTotalEffort)
    {
        double avgTotalEffort = 0;
        if (_isNearObject)
        {
            // the average total effort over the last gMemorySize (at most) turns we were on the object
            for (auto totEff: _totalEfforts)
                avgTotalEffort += totEff;
            avgTotalEffort /= _totalEfforts.size();
        }
        inputs.push_back(avgTotalEffort);
    }
    
    // how much did we contribute recently?
    double avgEffort = 0;
    if (!_efforts.empty())
    {
        for (auto eff: _efforts)
            avgEffort += eff;
        avgEffort /= _efforts.size();
    }
    inputs.push_back(avgEffort);
    
    return inputs;
}

void CoopFixed2Controller::stepController()
{

    // ---- compute and read out ----

    _NN->setWeights(_parameters); // set-up NN

    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values

    _NN->setInputs(inputs);

    _NN->step();

    std::vector<double> outputs = _NN->readOut();

    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;

    if (_canMove)
    {
        _wm->_desiredTranslationalValue = (outputs[0] + 1) / 2; // outputs[0] between [-1, 1], velocity : [0, 1]
        _wm->_desiredRotationalVelocity = outputs[1];
    }
    else
    {
        _wm->_desiredTranslationalValue = 1; //bump into the coop
        _wm->_desiredRotationalVelocity = 0;
    }
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;

    // Effort value
    if (CoopFixed2SharedData::gFixedEffort)
    {
        //Introduce a fixed level of cooperation for all robots so we focus on choosing the optimal number of partners
        _wm->_cooperationLevel = CoopFixed2SharedData::gFixedEffortValue;
    }
    else
    {
        // Introduce fixed cooperation levels for the last gNbFakeRobots robots
        int nbTrueRobots = gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots;
        if (_wm->getId() < nbTrueRobots)
            _wm->_cooperationLevel = (outputs[2]+1.0); // in [0, 2]
        else
            _wm->_cooperationLevel = (double)(_wm->getId()-nbTrueRobots)/(double)CoopFixed2SharedData::gNbFakeRobots * CoopFixed2SharedData::gFakeCoopValue;
    }
}


void CoopFixed2Controller::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    delete _NN;
    
    switch ( CoopFixed2SharedData::gControllerType )
    {
        case 0:
        {
            _NN = new MLP(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        case 1:
        {
            _NN = new Perceptron(_parameters, _nbInputs, _nbOutputs);
            
            break;
        }
        case 2:
        {
            _NN = new Elman(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        default: // default: no controller
            std::cerr << "[ERROR] gController type unknown (value: " << CoopFixed2SharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int CoopFixed2Controller::computeRequiredNumberOfWeights()
{
    return _NN->getRequiredNumberOfWeights();
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void CoopFixed2Controller::performVariation()
{
    if ( CoopFixed2SharedData::gIndividualMutationRate > ranf() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( CoopFixed2SharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(CoopFixed2SharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << CoopFixed2SharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
}

void CoopFixed2Controller::mutateGaussian(double sigma) // mutate within bounds.
{
    _currentSigma = sigma;
    
    for (double &curWeight : _currentGenome)
    {
        double value = curWeight + getGaussianRand(0,_currentSigma);
        // bouncing upper/lower bounds
        if ( value < _minValue )
        {
            double range = _maxValue - _minValue;
            double overflow = - (value - _minValue );
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _minValue + overflow;
            else // overflow btw range and range*2
                value = _minValue + range - (overflow-range);
        }
        else if ( value > _maxValue )
        {
            double range = _maxValue - _minValue;
            double overflow = value - _maxValue;
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _maxValue - overflow;
            else // overflow btw range and range*2
                value = _maxValue - range + (overflow-range);
        }

        curWeight = value;
    }
}


void CoopFixed2Controller::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(rand()%100) / 100.f; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}


void CoopFixed2Controller::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs )
    {
        _nbInputs = (1+1+1+1) * _wm->_cameraSensorsNb; // isItAnAgent? + isItAWall? + isItAnObject + nbNearbyRobots
    }
    
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    
    _nbInputs += 1; // how many robots around?
    
    if (CoopFixed2SharedData::gTotalEffort)
        _nbInputs += 1; // what's the total effort given to the object?
    
    _nbInputs += 1; // how much did we contribute?
    
    // wrt outputs
    
    _nbOutputs = 2+1; // 2 outputs for movement + 1 for cooperation
}

void CoopFixed2Controller::initController()
{
    _nbHiddenLayers = CoopFixed2SharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = CoopFixed2SharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();
    
    unsigned int nbGenes = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush;
    
    _currentGenome.clear();
    _currentGenome.reserve(nbGenes);

    // Intialize genomes
    for ( unsigned int i = 0 ; i < nbGenes; i++ )
    {
        _currentGenome.push_back((ranf()*2.0)-1.0); // weights: random init between -1 and +1
    }
    
    updatePhenotype();
    
    // state variables
    _nbNearbyRobots = 0;
    _isNearObject = false;
    _efforts.clear();
    _totalEfforts.clear();
}

void CoopFixed2Controller::reset()
{
    initController();
    resetFitness();
}


void CoopFixed2Controller::mutateSigmaValue()
{
    double dice = ranf();
    
    if ( dice <= CoopFixed2SharedData::gProbaMutation )
    {
        dice = ranf();
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + CoopFixed2SharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > CoopFixed2SharedData::gSigmaMax)
            {
                _currentSigma = CoopFixed2SharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - CoopFixed2SharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < CoopFixed2SharedData::gSigmaMin )
            {
                _currentSigma = CoopFixed2SharedData::gSigmaMin;
            }
        }
    }
}

void CoopFixed2Controller::loadNewGenome( genome __newGenome )
{
    _currentGenome = __newGenome.first;
    _currentSigma = __newGenome.second;
    performVariation();
    updatePhenotype();
}

void CoopFixed2Controller::updatePhenotype()
{
    // could be more complicated!
    _parameters = _currentGenome;
}

double CoopFixed2Controller::getFitness()
{
    // nothing to do
    return _wm->_fitnessValue;
}

/*
 * note: resetFitness is first called by the Controller's constructor.
 */
void CoopFixed2Controller::resetFitness()
{
    updateFitness(0);
}

void CoopFixed2Controller::updateFitness( double __newFitness )
{
    if (__newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    _wm->_fitnessValue = __newFitness;
}

void CoopFixed2Controller::increaseFitness( double __delta )
{
    updateFitness(_wm->_fitnessValue+__delta);
}

void CoopFixed2Controller::wasNearObject(double __totalInvest, double __invest, int __nbRobots )
{

    if (_wm->getId() < gNbOfRobots - CoopFixed2SharedData::gNbFakeRobots) // Green LED because we're a true robot (and active)
        _wm->setRobotLED_colorValues(0x32, 0xCD, 0x32);
    else // Red LED because we're a fake robot
        _wm->setRobotLED_colorValues(0xFF, 0x00, 0x7F);
    
    _isNearObject = true;
    _nbNearbyRobots = __nbRobots;
    
    double coeff = CoopFixed2SharedData::gConstantK/(1.0+pow(__nbRobots-2, 2)); // \frac{k}{1+(n-2)^2}
    double payoff = coeff * pow(__totalInvest, CoopFixed2SharedData::gConstantA) - __invest;

    increaseFitness(payoff);
    _efforts.push_back(__invest);
    if (_efforts.size() >= CoopFixed2SharedData::gMemorySize)
        _efforts.pop_front();

    _totalEfforts.push_back(__totalInvest);
    if (_totalEfforts.size() >= CoopFixed2SharedData::gMemorySize)
        _totalEfforts.pop_front();
}

void CoopFixed2Controller::dumpGenome()
{
    std::cout <<"Dumping genome of robot #" << _wm->getId() << std::endl;
    std::cout << _currentSigma << " ";
    std::cout << _currentGenome.size() << " ";
    for (auto gene: _currentGenome)
        std::cout << gene << " ";
    std::cout << std::endl;
}

void CoopFixed2Controller::setCanMove(bool _canMove) {
    CoopFixed2Controller::_canMove = _canMove;
}

std::string CoopFixed2Controller::inspect()
{
    std::stringstream out;
    out << "Near object: " << ((_isNearObject)? "True" : "False") << ".\n";
    if (_isNearObject)
    {
        out << std::setprecision(3);
        out << "\tLast cooperation value: " << _wm->_cooperationLevel << ".\n";
        out << "\tTotal Effort history: ";
        for (auto curTotEffort : _totalEfforts)
        {
            out << curTotEffort << ", ";
        }
        out << ".\n";
        out << "\tOwn Effort history: ";
        for (auto curEffort : _efforts)
        {
            out << curEffort << ", ";
        }
        out << ".\n";
    }
    std::set<int> seen;
    for (int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        seen.insert(static_cast<int &&>(_wm->getObjectIdFromCameraSensor(i)));
    }
    out << "Seen objects:\n";
    for (int entityId : seen)
    {
        if (entityId == 0)
        {
            out << "\tA wall\n";
        }
        else if (Agent::isInstanceOf(entityId))
        {
            out << "\tAnother agent\n";
        }
        else if (entityId >= gPhysicalObjectIndexStartOffset)
        {
            out << "\tA cooperation opportunity ";
            auto coop = dynamic_cast<CoopFixed2OpportunityObj *>(gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            out << "with " << coop->getNbNearbyRobots() << " robots nearby.\n ";
        }
    }
    out << "next to " << _nbNearbyRobots << " robots.\n";
    out << "Actual fitness: " << getFitness() << "\n";
    return out.str();
}
