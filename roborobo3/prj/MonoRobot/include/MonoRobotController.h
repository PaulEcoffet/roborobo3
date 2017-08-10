/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#ifndef MONOROBOTCONTROLLER_H
#define MONOROBOTCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Graphics.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "MonoRobot/include/MonoRobotAgentObserver.h"
#include <neuralnetworks/NeuralNetwork.h>

#include <iomanip>
#include <utility>
#include <deque>

using namespace Neural;

class MonoRobotController : public Controller
{
protected:
    int _iteration;
    int _birthdate; // evaluation when this controller was initialized.
        
    bool _isListening;
    int _notListeningDelay;
    int _listeningDelay;
        
    std::vector<double> _parameters;
    std::string _nnType;
    std::vector<int> _nbHiddenNeuronsPerLayer;
    std::vector<int> _nbBiasNeuronsPerLayer;
    NeuralNetwork* _NN;
    
    void createNN();
    
    //bool _isAlive; // agent stand still if not.
    
    void selectRandomGenome();
    void selectFirstGenome();
    
    void mutateGaussian( float sigma );
    void mutateUniform();
    
    void mutateSigmaValue();
    
    std::vector<double> getInputs();
    void setIOcontrollerSize();
    
    virtual void initController(); // called by resetRobot
    virtual void stepController();
    
    void stepEvolution();
    
    void updatePhenotype(); // updates the neural network when the genome is changed
    
    unsigned int computeRequiredNumberOfWeights();
    
    //        void setAliveStatus( bool isAlive ) { _isAlive = isAlive; }
            
    // current genome
    
    std::vector<double> _currentGenome;
    float _currentSigma;
    
    // other neural network inputs
    
    bool _isNearObject; // are we near an object
    int _nearbyObjectId; // the ID of the object nearby
	int _nbNearbyRobots; // number of robots on the footprint of the same object as us
    std::deque<double> _efforts; // average of how much we pushed
    std::deque<double> _totalEfforts; // average of how much everyone pushed
    
    // ANN
    double _minValue;
    double _maxValue;
    unsigned int _nbInputs;
    unsigned int _nbOutputs;
    unsigned int _nbHiddenLayers;
    std::vector<unsigned int>* _nbNeuronsPerHiddenLayer;
    
    // logging purpose
    double _Xinit;
    double _Yinit;
    double _dSumTravelled;
    
    void reset();
    
    virtual void logCurrentState();
    
    virtual void performVariation();
    
    virtual void resetFitness();
    virtual void updateFitness( double __newFitness );

    
public:
    
    typedef std::pair<std::vector<double>, double> genome;
    
    MonoRobotController(RobotWorldModel *wm);
    ~MonoRobotController();
    
    void step();
    
    int getBirthdate() { return _birthdate; }
    
    bool isListening() { return _isListening; }
    
    genome getGenome() { return std::make_pair(_currentGenome, _currentSigma); }
    
    void loadNewGenome( genome __newGenome, bool __mutate );
    
    virtual double getFitness();
    
    double getCooperationLevel() { return _wm->_cooperationLevel; }
    
    void increaseFitness( double __delta );
    
    void wasNearObject( int __objectId, bool __objectDidMove, double __totalEffort, double __effort, int __nbRobots ); // callback from the object whose footprint we're on, telling us how much it moved
    
    void dumpGenome();
};


#endif

