/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#ifndef MOVINGNSCONTROLLER_H
#define MOVINGNSCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Graphics.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingNS/include/MovingNSAgentObserver.h"
#include <neuralnetworks/NeuralNetwork.h>

#include <iomanip>
#include <utility>

using namespace Neural;

typedef std::pair<std::vector<double>, double> genome;


class MovingNSController : public Controller
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
    NeuralNetwork* nn;
    
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
    
    // fitness memory
    
    double _lastFitnesses[5]; // fitness we gained on the last 5 rounds
    bool _lastPushTries[5]; // did we try to push recently
    bool _isNearObject; //are we near an object
    
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
    
    MovingNSController(RobotWorldModel *wm);
    ~MovingNSController();
    
    void step();
    
    int getBirthdate() { return _birthdate; }
    
    bool isListening() { return _isListening; }
    
    genome getGenome() { return std::make_pair(_currentGenome, _currentSigma); }
    
    void loadNewGenome( genome __newGenome );
    
    virtual double getFitness();
    
    void increaseFitness( double __delta );
    
    virtual void updatePushes();
    
    void wasNearObject( bool __didMove, double __movement ); // callback from the object whose footprint we're on, telling us how much it moved
    
};


#endif

