/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 *
 */



#ifndef COOPFIXED2SHAREDDATA_H
#define COOPFIXED2SHAREDDATA_H

class CoopFixed2SharedData {
    
public:
    
    // -----
    
    static double gSigmaMin; //! used with gDynamicSigma defined to true
    static double gUpdateSigmaStep; //!step used in the drecrease or increas of the value of sigma
    static double gSigmaRef; //! reference value of sigma
    static double gSigmaMax; //! maximal value of sigma
    static double gProbaMutation; //! probability of transmitting the current genome mutated with sigma ref
    static int gEvaluationTime; //! theoretical duration of a generation (ie. maximum time a controller will be evaluated on a robot)

    static double gMonitorPositions; //! used in WorldObserver. Compute and log all necessary information for monitoring position and orientation wrt. center.
    
    static bool gPropertiesLoaded;
    
    static int gNbHiddenLayers; // default: 1
    static int gNbNeuronsPerHiddenLayer; // default: 5

    static bool gSnapshots; // take snapshots
    static int gSnapshotsFrequency; // every N generations
    
    static int gControllerType; // controller type (0: MLP, 1: Perceptron, 2: Elman)

    static double gIndividualMutationRate;
    
    static int gMutationOperator;
    
    static double gSigma;

	static int gMemorySize;
    
    static bool gTotalEffort; // add total effort as an input to the NN
    
    static double gFakeCoopValue;
    static int gNbFakeRobots;
    
    static bool gFixedEffort;
    static double gFixedEffortValue;
    
    static int gGenerationLog; // take videos and log genomes every Nth generation
    static bool gTakeVideo; // take videos or not?
    static bool gLogGenome;
    static bool gLogGenomeSnapshot;   // log only if it%gEvaluationTime*gSnapshotsFrequency=0

    
    
    // Parameters relative to the payoff function
    
    static double gConstantK;
    static double gConstantA;

        
    // -----
    
    
    
};


#endif
