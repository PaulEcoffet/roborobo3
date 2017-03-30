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

using namespace Neural;


class MovingNSController : public Controller
{
protected:
    int _iteration;
    int _birthdate; // evaluation when this controller was initialized.
        
    bool _isListening;
    int _notListeningDelay;
    int _listeningDelay;
    
    int _nbGenomeTransmission;
    
    std::vector<double> _parameters;
    std::string _nnType;
    std::vector<int> _nbHiddenNeuronsPerLayer;
    std::vector<int> _nbBiasNeuronsPerLayer;
    NeuralNetwork* nn;
    
    void createNN();
    
    //bool _isAlive; // agent stand still if not.
    bool _isNewGenome;
    
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
    
    virtual void broadcastGenome();
    
    void loadNewGenome();
    
    unsigned int computeRequiredNumberOfWeights();
    
    //        void setAliveStatus( bool isAlive ) { _isAlive = isAlive; }
    bool getNewGenomeStatus() { return _isNewGenome; }
    void setNewGenomeStatus( bool __status ) { _isNewGenome = __status; }
    
    // incoming genomes reservoir
    
    std::map< std::pair<int,int>, std::vector<double> > _genomesList;
    std::map< std::pair<int,int>, float > _sigmaList;
    std::map< std::pair<int,int>, float > _fitnessValuesList;
    
    // current genome
    
    std::vector<double> _currentGenome;
    float _currentSigma;
    
    // fitness memory
    
    double _lastFitnesses[5]; // fitness we gained on the last 5 rounds
    
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
    
    bool storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness=0);
    void reset();
    
    void clearReservoir(); // clear genomesList, sigmaList, fitnessesList and birthdayList
    
    virtual void logCurrentState();
    
    virtual void performSelection();
    virtual void performVariation();
    
    virtual void resetFitness();
    virtual void updateFitness();
    
public:
    
    MovingNSController(RobotWorldModel *wm);
    ~MovingNSController();
    
    void step();
    
    int getBirthdate() { return _birthdate; }
    
    bool isListening() { return _isListening; }
    
    virtual double getFitness();
    
    virtual void updateFitness( double __newFitness );
    
    
};


#endif

