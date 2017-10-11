#!/bin/env python3

import sys
from random import randint
from math import ceil

def main():
    if len(sys.argv) < 5:
        print("Usage: ./compartment_generator.py nb_robots_per_comp nb_obj_per_comp nb_total_robots nb_fakes")
        sys.exit(-1)
    nb_robots_per_comp = int(sys.argv[1])
    nb_obj_per_comp = int(sys.argv[2])
    nb_total_robots = int(sys.argv[3])
    nb_fakes = int(sys.argv[4])
    if nb_total_robots % nb_robots_per_comp != 0:
        print("WARNING: not same amount of robots per compartment")

    nb_compartments = ceil(nb_total_robots / nb_robots_per_comp)
    nb_total_obj = nb_compartments * nb_obj_per_comp
    conf = get_default().format(nb_robots=nb_total_robots, nb_objects=nb_total_obj, nb_fakes=nb_fakes)

    # Let's position the objects first
    cur_compartment = 0
    obj_coordinates = []
    for i in range(nb_total_obj):
        wrong_pos = True
        while wrong_pos:
            wrong_pos = False
            x = randint((cur_compartment % 5) * 208 + 30, ((cur_compartment % 5) + 1) * 208 - 30)
            y = randint((cur_compartment // 5) * 208 + 30, ((cur_compartment // 5) + 1) * 208 - 15)
            for (x0, y0) in obj_coordinates:
                if (x - x0)**2 + (y - y0)**2 <= 25**2:
                    print(i, "at wrong pos", file=sys.stderr)
                    wrong_pos = True
                    break
        obj_coordinates.append((x, y))
        conf += "physicalObject[{}].x = {}\n".format(i, x)
        conf += "physicalObject[{}].y = {}\n".format(i, y)
        cur_compartment = (cur_compartment + 1) % nb_compartments

    # And now the robots
    for i in range(nb_total_robots):
        wrong_pos = True
        while wrong_pos:
            wrong_pos = False
            x = randint((cur_compartment % 5) * 208 + 30, ((cur_compartment % 5) + 1) * 208 - 30)
            y = randint((cur_compartment // 5) * 208 + 30, ((cur_compartment // 5) + 1) * 208 - 30)
            for (x0, y0) in obj_coordinates:
                if (x - x0)**2 + (y - y0)**2 <= 15**2:
                    wrong_pos = True
                    break
        conf += "robot[{}].x = {}\n".format(i, x)
        conf += "robot[{}].y = {}\n".format(i, y)
        cur_compartment = (cur_compartment + 1) % nb_compartments

    print(conf)


def get_default():
    return """
gInitialNumberOfRobots = {nb_robots}
gNbOfPhysicalObjects = {nb_objects}
gNbOfLandmarks = 0

gExtendedSensoryInputs = true # Should be rewritten to suit your need. Check code.
gTotalEffort = true

gFakeCoopValue = 2.0
gNbFakeRobots = {nb_fakes}

gFixedEffort = false
gFixedEffortValue = 0.25

gControllerType = 2 # MLP=0, Perceptron=1, Elman=2

gSensorRange = 32
gSynchronization = true # not implemented

gDisplayMode = 2
gBatchMode = false

gRandomSeed = -1

gVerbose = true

# =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

gMaxIt = 20000000 #-1=infinite

gEvaluationTime = 2000
gEvaluationsPerGeneration = 1
gGenerationLog = 2000 # make a video of each n-th generation and register its genomes
gTakeVideo = true

gNotListeningStateDelay = 400			# -1: infinite  ; 0: no delay (default) ; >0: delay
gListeningStateDelay = -1					# -1: infinite (default) ; 0: no delay (inactive) ; >0: delay
						# remark 1: ignored if gNotListeningStateDelay=-1
						# remark 2: setting gNotListeningStateDelay>0 and gListeningStateDelay=0 is possible, but useless

gLimitGenomeTransmission = false		# optional: default is false
gMaxNbGenomeTransmission = 2    		# optional: (ignored if gLimitGenomeTransmission is false)
gSelectionMethod = 0							# optional: (default is 0) ; 0: random ; 1: first

gLogGenome = false
gLogGenomeSnapshot = true # only if it%gEvaluationTime*gSnapshotsFrequency=0

# =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#
# general file information
#

ConfigurationLoaderObjectName = MovingNSConfigurationLoader

gRobotMaskImageFilename = data/minirobot-mask.bmp
gRobotSpecsImageFilename = data/minirobot-specs-12sensors.bmp

gForegroundImageFilename = data/env_arena_environment_5_5.bmp
gEnvironmentImageFilename = data/env_arena_environment_5_5.bmp
gBackgroundImageFilename = data/env_arena_background_5_5.bmp
gFootprintImageFilename = data/env_arena_background_5_5.bmp

gNbLines = 5
gNbRows = 5


gScreenWidth = 1024
gScreenHeight = 1024

gBorderSize = 4
gZoneHeight = 200
gZoneWidth = 200

#gLogFilename = logs/log.txt # if commented, create a time-stamped file.
gLogCommentText = (under-development)

gSnapshots = true # take snapshots
gSnapshotsFrequency = 10 # every N generations


# =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#
# Other parameters
#

# general purpose

gPauseMode = false
gDisplaySensors = 0   # 0: no, 1: only-contact, 2: all + contacts are red, 3: all (same color)
gDisplayTail = false
gRobotDisplayFocus = false
gDisplayGroundCaption = false
gNiceRendering = true
SlowMotionMode = false
gUserCommandMode = false
gRobotLEDdisplay = true

gFastDisplayModeSpeed = 60
gFramesPerSecond = 60

gRobotIndexFocus = 0

gNumberOfRobotGroups = 1 # unused

gPhysicalObjectIndexStartOffset = 1
gRobotIndexStartOffset = 1048576  # 0x100000

gFootprintImage_restoreOriginal = true

# Monitoring

gVideoRecording = false # significantly slow down simulation
gTrajectoryMonitor = false  # significantly slow down simulation

gInspectorMode = false
gInspectorAgent = false
gMonitorRobot = false

# Inspector cursor (''god mode'')

gInspectorCursorHorizontalSpeed = 1
gInspectorCursorVerticalSpeed = 1
gInspectorAgentXStart = 1
gInspectorAgentYStart = 1

# robot dynamics and structure

gMaxTranslationalSpeed = 2  # value btw 0+ and robot width in pixels
gMaxTranslationalDeltaValue = 2 	# value btw 0+ and gMaxRotationalSpeed
gMaxRotationalSpeed = 30

gInspectorCursorMaxSpeedOnXaxis = 5
gInspectorCursorMaxSpeedOnYaxis = 10

gLocomotionMode = 0

# Neural networks

gNbHiddenLayers = 1
gNbNeuronsPerHiddenLayer = 10
gNeuronWeightRange = 800.0  # [-400,+400]

# =-=-=-=-=-=

# simulation parameters

gRadioNetwork = true
gMaxRadioDistance = 16  # not used. Assume proximity sensor distance.

gMonitorPositions = false # slow down if true.

# Max nb of trials for picking a random location for an object OR robot
# note: it may not be possible (or difficult) to pick a location.
#       this variable is to avoid infinite loop.
gLocationFinderMaxNbOfTrials = 1000 # 100?

# =-=-=-=-=-=

# parameters wrt. mEDEA

gIndividualMutationRate = 0.1 # apply (whatever) mutation operator? 1.0=always_mutate
gMutationOperator = 1 # 0: uniform, 1: gaussian with evolved sigma, 2: gaussian with fixed sigma
gSigmaMin = 0.01
gProbaMutation = 0.0
gUpdateSigmaStep = 0.35
gSigmaRef = 0.1
gSigmaMax = 0.5
gSigma=0.01 # only if mutation operator is set to 2

# =-=-=-=-=-=

# robot localisation

# gAgentsInitArea* constrains localization to the designated area.
# If not present, whole arena's area is considered
# Ignored if agent localization is explicit
gAgentsInitAreaX = 10
gAgentsInitAreaY = 10
gAgentsInitAreaWidth = 580
gAgentsInitAreaHeight = 580

# Robot energy

gEnergyLevel = false # false: energy is not used
gEnergyInit = 400
gEnergyMax = 400
gEnergyRequestOutput = true
gEnergyRefill = true # robot energy refill

# Landmarks

VisibleLandmarks = true
gLandmarkRadius = 10.0

# Physical objects

gPhysicalObjectsVisible = true
gPhysicalObjectsRedraw = false

# gPhysicalObjectsInitArea* constrains localization to the designated area.
# If not present, whole arena's area is considered (with a 10-pixel border)
# Ignored if object localization is explicit
gPhysicalObjectsInitAreaX = 75
gPhysicalObjectsInitAreaY = 75
gPhysicalObjectsInitAreaWidth = 450
gPhysicalObjectsInitAreaHeight = 450

gPhysicalObjectDefaultType = 5 # moving object
gPhysicalObjectDefaultRelocate = true
gPhysicalObjectDefaultOverwrite = false
gPhysicalObjectDefaultRadius = 8
gPhysicalObjectDefaultFootprintRadius = 0
gPhysicalObjectDefaultDisplayColorRed = 220
gPhysicalObjectDefaultDisplayColorGreen = 220
gPhysicalObjectDefaultDisplayColorBlue = 0
gPhysicalObjectDefaultSolid_w = 16
gPhysicalObjectDefaultSolid_h = 16
gPhysicalObjectDefaultSoft_w = 22
gPhysicalObjectDefaultSoft_h = 22

gPhysicalObjectDefaultRegrowTimeMax = 0

# Project-specific stuff

gMovableObjects = true
gStuckMovableObjects = true

# Payoff function : k*(sum of x)^a/(...) - x

gConstantA = 0.5
gConstantK = 1.414213


###################################
# Objects and robots positionning #
###################################

"""

main()
