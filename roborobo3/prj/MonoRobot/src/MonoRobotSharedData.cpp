/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#include "MonoRobot/include/MonoRobotSharedData.h"

double MonoRobotSharedData::gSigmaMin = 0.0;
double MonoRobotSharedData::gProbaMutation = 0.0;
double MonoRobotSharedData::gUpdateSigmaStep = 0.0;
double MonoRobotSharedData::gSigmaRef = 0.0; // reference value of sigma
double MonoRobotSharedData::gSigmaMax = 0.0; // maximal value of sigma
int MonoRobotSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool MonoRobotSharedData::gSynchronization = true;

bool MonoRobotSharedData::gEnergyRequestOutput = 1;

double MonoRobotSharedData::gMonitorPositions;

bool MonoRobotSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int MonoRobotSharedData::gNbHiddenLayers = 1;
int MonoRobotSharedData::gNbNeuronsPerHiddenLayer = 5;
int MonoRobotSharedData::gNeuronWeightRange = 800;

bool MonoRobotSharedData::gSnapshots = true; // take snapshots
int MonoRobotSharedData::gSnapshotsFrequency = 50; // every N generations

int MonoRobotSharedData::gControllerType = -1; // cf. header for description

bool MonoRobotSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int MonoRobotSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int MonoRobotSharedData::gSelectionMethod = 0; // default: random selection

int MonoRobotSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int MonoRobotSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool MonoRobotSharedData::gLogGenome = false;
bool MonoRobotSharedData::gLogGenomeSnapshot = false;

double MonoRobotSharedData::gIndividualMutationRate = 1.0;

int MonoRobotSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double MonoRobotSharedData::gSigma = 0.01; // 0.01 is just some random value.

constexpr int MonoRobotSharedData::gMemorySize;

bool MonoRobotSharedData::gTotalEffort = true;

int MonoRobotSharedData::gGenerationLog = 1000;
bool MonoRobotSharedData::gTakeVideo = true;

int MonoRobotSharedData::gNumberOfPeriods = 3;
int MonoRobotSharedData::gEvaluationsPerGeneration = 5;

int MonoRobotSharedData::gBorderSize = -1;
int MonoRobotSharedData::gZoneWidth = -1;
int MonoRobotSharedData::gZoneHeight = -1;
int MonoRobotSharedData::gNbLines = -1;
int MonoRobotSharedData::gNbRows = -1;

double MonoRobotSharedData::gConstantK = 0;
double MonoRobotSharedData::gConstantA = 0;
