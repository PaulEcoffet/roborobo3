/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#include "CoopFixed2/include/CoopFixed2SharedData.h"

double CoopFixed2SharedData::gSigmaMin = 0.0;
double CoopFixed2SharedData::gProbaMutation = 0.0;
double CoopFixed2SharedData::gUpdateSigmaStep = 0.0;
double CoopFixed2SharedData::gSigmaRef = 0.0; // reference value of sigma
double CoopFixed2SharedData::gSigmaMax = 0.0; // maximal value of sigma
int CoopFixed2SharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool CoopFixed2SharedData::gSynchronization = true;

bool CoopFixed2SharedData::gEnergyRequestOutput = 1;

double CoopFixed2SharedData::gMonitorPositions;

bool CoopFixed2SharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int CoopFixed2SharedData::gNbHiddenLayers = 1;
int CoopFixed2SharedData::gNbNeuronsPerHiddenLayer = 5;
int CoopFixed2SharedData::gNeuronWeightRange = 800;

bool CoopFixed2SharedData::gSnapshots = true; // take snapshots
int CoopFixed2SharedData::gSnapshotsFrequency = 50; // every N generations

int CoopFixed2SharedData::gControllerType = -1; // cf. header for description

bool CoopFixed2SharedData::gLimitGenomeTransmission = false; // default: do not limit.
int CoopFixed2SharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int CoopFixed2SharedData::gSelectionMethod = 0; // default: random selection

int CoopFixed2SharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int CoopFixed2SharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

double CoopFixed2SharedData::gIndividualMutationRate = 1.0;

int CoopFixed2SharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double CoopFixed2SharedData::gSigma = 0.01; // 0.01 is just some random value.

int CoopFixed2SharedData::gMemorySize = 20;

bool CoopFixed2SharedData::gTotalEffort = true;

double CoopFixed2SharedData::gFakeCoopValue = 2.0; // maximum value of ...
int CoopFixed2SharedData::gNbFakeRobots = 10; // number of fixed-coop robots in the pop

bool CoopFixed2SharedData::gFixedEffort = false;
double CoopFixed2SharedData::gFixedEffortValue = 0.25;

int CoopFixed2SharedData::gGenerationLog = 5000;
bool CoopFixed2SharedData::gTakeVideo = false;
bool CoopFixed2SharedData::gLogGenome = false;
bool CoopFixed2SharedData::gLogGenomeSnapshot = false;

double CoopFixed2SharedData::gConstantK = 0;
double CoopFixed2SharedData::gConstantA = 0;

