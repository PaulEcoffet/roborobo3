/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#include "CoopOpportunity2Max/include/CoopOpportunity2MaxSharedData.h"

double CoopOpportunity2MaxSharedData::gSigmaMin = 0.0;
double CoopOpportunity2MaxSharedData::gProbaMutation = 0.0;
double CoopOpportunity2MaxSharedData::gUpdateSigmaStep = 0.0;
double CoopOpportunity2MaxSharedData::gSigmaRef = 0.0; // reference value of sigma
double CoopOpportunity2MaxSharedData::gSigmaMax = 0.0; // maximal value of sigma
int CoopOpportunity2MaxSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool CoopOpportunity2MaxSharedData::gSynchronization = true;

bool CoopOpportunity2MaxSharedData::gEnergyRequestOutput = 1;

double CoopOpportunity2MaxSharedData::gMonitorPositions;

bool CoopOpportunity2MaxSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int CoopOpportunity2MaxSharedData::gNbHiddenLayers = 1;
int CoopOpportunity2MaxSharedData::gNbNeuronsPerHiddenLayer = 5;
int CoopOpportunity2MaxSharedData::gNeuronWeightRange = 800;

bool CoopOpportunity2MaxSharedData::gSnapshots = true; // take snapshots
int CoopOpportunity2MaxSharedData::gSnapshotsFrequency = 50; // every N generations

int CoopOpportunity2MaxSharedData::gControllerType = -1; // cf. header for description

bool CoopOpportunity2MaxSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int CoopOpportunity2MaxSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int CoopOpportunity2MaxSharedData::gSelectionMethod = 0; // default: random selection

int CoopOpportunity2MaxSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int CoopOpportunity2MaxSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

double CoopOpportunity2MaxSharedData::gIndividualMutationRate = 1.0;

int CoopOpportunity2MaxSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double CoopOpportunity2MaxSharedData::gSigma = 0.01; // 0.01 is just some random value.

int CoopOpportunity2MaxSharedData::gMemorySize = 20;

bool CoopOpportunity2MaxSharedData::gTotalEffort = true;

double CoopOpportunity2MaxSharedData::gFakeCoopValue = 2.0; // maximum value of ...
int CoopOpportunity2MaxSharedData::gNbFakeRobots = 10; // number of fixed-coop robots in the pop

bool CoopOpportunity2MaxSharedData::gFixedEffort = false;
double CoopOpportunity2MaxSharedData::gFixedEffortValue = 0.25;

int CoopOpportunity2MaxSharedData::gGenerationLog = 5000;
bool CoopOpportunity2MaxSharedData::gTakeVideo = false;
bool CoopOpportunity2MaxSharedData::gLogGenome = false;
bool CoopOpportunity2MaxSharedData::gLogGenomeSnapshot = false;

double CoopOpportunity2MaxSharedData::gConstantK = 0;
double CoopOpportunity2MaxSharedData::gConstantA = 0;

