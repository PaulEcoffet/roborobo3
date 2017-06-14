/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#include "MovingNS/include/MovingNSSharedData.h"

double MovingNSSharedData::gSigmaMin = 0.0;
double MovingNSSharedData::gProbaMutation = 0.0;
double MovingNSSharedData::gUpdateSigmaStep = 0.0;
double MovingNSSharedData::gSigmaRef = 0.0; // reference value of sigma
double MovingNSSharedData::gSigmaMax = 0.0; // maximal value of sigma
int MovingNSSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool MovingNSSharedData::gSynchronization = true;

bool MovingNSSharedData::gEnergyRequestOutput = 1;

double MovingNSSharedData::gMonitorPositions;

bool MovingNSSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int MovingNSSharedData::gNbHiddenLayers = 1;
int MovingNSSharedData::gNbNeuronsPerHiddenLayer = 5;
int MovingNSSharedData::gNeuronWeightRange = 800;

bool MovingNSSharedData::gSnapshots = true; // take snapshots
int MovingNSSharedData::gSnapshotsFrequency = 50; // every N generations

int MovingNSSharedData::gControllerType = -1; // cf. header for description

bool MovingNSSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int MovingNSSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int MovingNSSharedData::gSelectionMethod = 0; // default: random selection

int MovingNSSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int MovingNSSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool MovingNSSharedData::gLogGenome = false;
bool MovingNSSharedData::gLogGenomeSnapshot = false;

double MovingNSSharedData::gIndividualMutationRate = 1.0;

int MovingNSSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double MovingNSSharedData::gSigma = 0.01; // 0.01 is just some random value.

constexpr int MovingNSSharedData::gMemorySize;

double MovingNSSharedData::gConstantK = 0;
double MovingNSSharedData::gConstantA = 0;

