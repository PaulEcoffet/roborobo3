cmake_minimum_required(VERSION 3.5)
project(roborobo3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -O2")

set(SOURCE_FILES
        include/contrib/neuralnetworks/Elman.h
        include/contrib/neuralnetworks/LayeredNeuralNetwork.h
        include/contrib/neuralnetworks/MLP.h
        include/contrib/neuralnetworks/NeuralNetwork.h
        include/contrib/neuralnetworks/NeuralNetworkException.h
        include/contrib/neuralnetworks/Perceptron.h
        include/contrib/neuralnetworks/ReservoirNeuralNetwork.h
        include/contrib/zsu/func.h
        include/contrib/zsu/Properties.h
        include/contrib/zsu/StringUtils.h
        include/contrib/zsu/strip.h
        include/contrib/json/json.hpp
        include/core/Agents/Agent.h
        include/core/Agents/InspectorAgent.h
        include/core/Agents/Robot.h
        include/core/Config/GlobalConfigurationLoader.h
        include/core/Controllers/Controller.h
        include/core/Observers/AgentObserver.h
        include/core/Observers/Observer.h
        include/core/Observers/WorldObserver.h
        include/core/RoboroboMain/common.h
        include/core/RoboroboMain/main.h
        include/core/RoboroboMain/roborobo.h
        include/core/Utilities/ExtendedProperties.h
        include/core/Utilities/Geometry.h
        include/core/Utilities/Graphics.h
        include/core/Utilities/LogManager.h
        include/core/Utilities/Misc.h
        include/core/Utilities/SDLMain.h
        include/core/Utilities/Timer.h
        include/core/World/CircleObject.h
        include/core/World/PhysicalObject.h
        include/core/World/SquareObject.h
        include/core/World/World.h
        include/core/WorldModels/RobotWorldModel.h
        include/core/WorldModels/WorldModel.h
        include/ext/Config/ConfigurationLoader.h
        include/ext/Config/CoopOpportunity2MaxConfigurationLoader.h
        include/ext/Config/MonoRobotConfigurationLoader.h
        include/ext/Config/MovingEEConfigurationLoader.h
        include/ext/Config/MovingNSConfigurationLoader.h
        include/ext/Config/SingleGenomeConfigurationLoader.h
        include/ext/Config/TemplateBoidsConfigurationLoader.h
        include/ext/Config/TemplateEEConfigurationLoader.h
        include/ext/Config/TemplateMedeaConfigurationLoader.h
        include/ext/Config/TemplateMovingConfigurationLoader.h
        include/ext/Config/TemplateRandomwalkConfigurationLoader.h
        include/ext/Config/TemplateVanillaEEConfigurationLoader.h
        include/ext/Config/TemplateWanderConfigurationLoader.h
        include/ext/World/EnergyItem.h
        include/ext/World/GateObject.h
        include/ext/World/LandmarkObject.h
        include/ext/World/MovableObject.h
        include/ext/World/MovingObject.h
        include/ext/World/PhysicalObjectFactory.h
        include/ext/World/RoundObject.h
        include/ext/World/SwitchObject.h
        prj/CoopOpportunity2Max/include/CoopOpportunity2MaxAgentObserver.h
        prj/CoopOpportunity2Max/include/CoopOpportunity2MaxController.h
        prj/CoopOpportunity2Max/include/CoopOpportunity2MaxSharedData.h
        prj/CoopOpportunity2Max/include/CoopOpportunity2MaxWorldObserver.h
        prj/CoopOpportunity2Max/include/CoopOpportunity2MaxMovingObject2Max.h
        prj/CoopOpportunity2Max/src/CoopOpportunity2MaxAgentObserver.cpp
        prj/CoopOpportunity2Max/src/CoopOpportunity2MaxController.cpp
        prj/CoopOpportunity2Max/src/CoopOpportunity2MaxSharedData.cpp
        prj/CoopOpportunity2Max/src/CoopOpportunity2MaxWorldObserver.cpp
        prj/CoopOpportunity2Max/src/CoopOpportunity2MaxMovingObject2Max.cpp
        prj/MonoRobot/include/MonoRobotAgentObserver.h
        prj/MonoRobot/include/MonoRobotController.h
        prj/MonoRobot/include/MonoRobotSharedData.h
        prj/MonoRobot/include/MonoRobotWorldObserver.h
        prj/MonoRobot/src/MonoRobotAgentObserver.cpp
        prj/MonoRobot/src/MonoRobotController.cpp
        prj/MonoRobot/src/MonoRobotSharedData.cpp
        prj/MonoRobot/src/MonoRobotWorldObserver.cpp
        prj/MovingEE/include/MovingEEAgentObserver.h
        prj/MovingEE/include/MovingEEController.h
        prj/MovingEE/include/MovingEESharedData.h
        prj/MovingEE/include/MovingEEWorldObserver.h
        prj/MovingEE/src/MovingEEAgentObserver.cpp
        prj/MovingEE/src/MovingEEController.cpp
        prj/MovingEE/src/MovingEESharedData.cpp
        prj/MovingEE/src/MovingEEWorldObserver.cpp
        prj/MovingNS/include/MovingNSAgentObserver.h
        prj/MovingNS/include/MovingNSController.h
        prj/MovingNS/include/MovingNSSharedData.h
        prj/MovingNS/include/MovingNSWorldObserver.h
        prj/MovingNS/src/MovingNSAgentObserver.cpp
        prj/MovingNS/src/MovingNSController.cpp
        prj/MovingNS/src/MovingNSSharedData.cpp
        prj/MovingNS/src/MovingNSWorldObserver.cpp
        prj/SingleGenome/include/SingleGenomeAgentObserver.h
        prj/SingleGenome/include/SingleGenomeController.h
        prj/SingleGenome/include/SingleGenomeSharedData.h
        prj/SingleGenome/include/SingleGenomeWorldObserver.h
        prj/SingleGenome/src/SingleGenomeAgentObserver.cpp
        prj/SingleGenome/src/SingleGenomeController.cpp
        prj/SingleGenome/src/SingleGenomeSharedData.cpp
        prj/SingleGenome/src/SingleGenomeWorldObserver.cpp
        prj/TemplateBoids/include/TemplateBoidsAgentObserver.h
        prj/TemplateBoids/include/TemplateBoidsController.h
        prj/TemplateBoids/include/TemplateBoidsSharedData.h
        prj/TemplateBoids/include/TemplateBoidsWorldObserver.h
        prj/TemplateBoids/src/TemplateBoidsAgentObserver.cpp
        prj/TemplateBoids/src/TemplateBoidsController.cpp
        prj/TemplateBoids/src/TemplateBoidsSharedData.cpp
        prj/TemplateBoids/src/TemplateBoidsWorldObserver.cpp
        prj/TemplateEE/include/TemplateEEAgentObserver.h
        prj/TemplateEE/include/TemplateEEController.h
        prj/TemplateEE/include/TemplateEESharedData.h
        prj/TemplateEE/include/TemplateEEWorldObserver.h
        prj/TemplateEE/src/TemplateEEAgentObserver.cpp
        prj/TemplateEE/src/TemplateEEController.cpp
        prj/TemplateEE/src/TemplateEESharedData.cpp
        prj/TemplateEE/src/TemplateEEWorldObserver.cpp
        prj/TemplateMedea/include/TemplateMedeaAgentObserver.h
        prj/TemplateMedea/include/TemplateMedeaController.h
        prj/TemplateMedea/include/TemplateMedeaSharedData.h
        prj/TemplateMedea/include/TemplateMedeaWorldObserver.h
        prj/TemplateMedea/src/TemplateMedeaAgentObserver.cpp
        prj/TemplateMedea/src/TemplateMedeaController.cpp
        prj/TemplateMedea/src/TemplateMedeaSharedData.cpp
        prj/TemplateMedea/src/TemplateMedeaWorldObserver.cpp
        prj/TemplateMoving/include/TemplateMovingAgentObserver.h
        prj/TemplateMoving/include/TemplateMovingController.h
        prj/TemplateMoving/include/TemplateMovingWorldObserver.h
        prj/TemplateMoving/src/TemplateMovingAgentObserver.cpp
        prj/TemplateMoving/src/TemplateMovingController.cpp
        prj/TemplateMoving/src/TemplateMovingWorldObserver.cpp
        prj/TemplateRandomwalk/include/TemplateRandomwalkAgentObserver.h
        prj/TemplateRandomwalk/include/TemplateRandomwalkController.h
        prj/TemplateRandomwalk/include/TemplateRandomwalkWorldObserver.h
        prj/TemplateRandomwalk/src/TemplateRandomwalkAgentObserver.cpp
        prj/TemplateRandomwalk/src/TemplateRandomwalkController.cpp
        prj/TemplateRandomwalk/src/TemplateRandomwalkWorldObserver.cpp
        prj/TemplateVanillaEE/include/TemplateVanillaEEAgentObserver.h
        prj/TemplateVanillaEE/include/TemplateVanillaEEController.h
        prj/TemplateVanillaEE/include/TemplateVanillaEESharedData.h
        prj/TemplateVanillaEE/include/TemplateVanillaEEWorldObserver.h
        prj/TemplateVanillaEE/src/TemplateVanillaEEAgentObserver.cpp
        prj/TemplateVanillaEE/src/TemplateVanillaEEController.cpp
        prj/TemplateVanillaEE/src/TemplateVanillaEESharedData.cpp
        prj/TemplateVanillaEE/src/TemplateVanillaEEWorldObserver.cpp
        prj/TemplateWander/include/TemplateWanderAgentObserver.h
        prj/TemplateWander/include/TemplateWanderController.h
        prj/TemplateWander/include/TemplateWanderWorldObserver.h
        prj/TemplateWander/src/TemplateWanderAgentObserver.cpp
        prj/TemplateWander/src/TemplateWanderController.cpp
        prj/TemplateWander/src/TemplateWanderWorldObserver.cpp
        src/contrib/neuralnetworks/Elman.cpp
        src/contrib/neuralnetworks/LayeredNeuralNetwork.cpp
        src/contrib/neuralnetworks/MLP.cpp
        src/contrib/neuralnetworks/NeuralNetwork.cpp
        src/contrib/neuralnetworks/NeuralNetworkException.cpp
        src/contrib/neuralnetworks/Perceptron.cpp
        src/contrib/neuralnetworks/ReservoirNeuralNetwork.cpp
        src/contrib/zsu/Properties.cpp
        src/contrib/zsu/StringUtils.cpp
        src/core/Agent.cpp
        src/core/AgentObserver.cpp
        src/core/CircleObject.cpp
        src/core/Controller.cpp
        src/core/ExtendedProperties.cpp
        src/core/Geometry.cpp
        src/core/Graphics.cpp
        src/core/InspectorAgent.cpp
        src/core/LogManager.cpp
        src/core/main.cpp
        src/core/Misc.cpp
        src/core/Observer.cpp
        src/core/PhysicalObject.cpp
        src/core/roborobo.cpp
        src/core/Robot.cpp
        src/core/RobotWorldModel.cpp
        src/core/SquareObject.cpp
        src/core/Timer.cpp
        src/core/World.cpp
        src/core/WorldModel.cpp
        src/core/WorldObserver.cpp
        src/ext/ConfigurationLoader.cpp
        src/ext/CoopOpportunity2MaxConfigurationLoader.cpp
        src/ext/EnergyItem.cpp
        src/ext/GateObject.cpp
        src/ext/LandmarkObject.cpp
        src/ext/MonoRobotConfigurationLoader.cpp
        src/ext/MovableObject.cpp
        src/ext/MovingEEConfigurationLoader.cpp
        src/ext/MovingNSConfigurationLoader.cpp
        src/ext/MovingObject.cpp
        src/ext/PhysicalObjectFactory.cpp
        src/ext/RoundObject.cpp
        src/ext/SingleGenomeConfigurationLoader.cpp
        src/ext/SwitchObject.cpp
        src/ext/TemplateBoidsConfigurationLoader.cpp
        src/ext/TemplateEEConfigurationLoader.cpp
        src/ext/TemplateMedeaConfigurationLoader.cpp
        src/ext/TemplateMovingConfigurationLoader.cpp
        src/ext/TemplateRandomwalkConfigurationLoader.cpp
        src/ext/TemplateVanillaEEConfigurationLoader.cpp
        src/ext/TemplateWanderConfigurationLoader.cpp
        src/ext/CoopFixed2ConfigurationLoader.cpp
        prj/CoopFixed2/include/CoopFixed2AgentObserver.h
        prj/CoopFixed2/include/CoopFixed2Controller.h
        prj/CoopFixed2/include/CoopFixed2WorldObserver.h
        prj/CoopFixed2/include/CoopFixed2SharedData.h
        prj/CoopFixed2/include/CoopFixed2Opportunity.h
        prj/CoopFixed2/include/CoopFixed2WorldModel.h
        prj/CoopFixed2/src/CoopFixed2AgentObserver.cpp
        prj/CoopFixed2/src/CoopFixed2Controller.cpp
        prj/CoopFixed2/src/CoopFixed2SharedData.cpp
        prj/CoopFixed2/src/CoopFixed2WorldObserver.cpp
        prj/CoopFixed2/src/CoopFixed2Opportunity.cpp
        prj/CoopFixed2/src/CoopFixed2WorldModel.cpp
        prj/PartnerChoice/src/PartnerChoiceController.cpp
        prj/PartnerChoice/include/PartnerChoiceController.h
        prj/PartnerChoice/src/PartnerChoiceAgentObserver.cpp
        prj/PartnerChoice/include/PartnerChoiceAgentObserver.h
        prj/PartnerChoice/src/PartnerChoiceWorldObserver.cpp
        prj/PartnerChoice/include/PartnerChoiceWorldObserver.h
        prj/PartnerChoice/src/PartnerChoiceSharedData.cpp
        prj/PartnerChoice/include/PartnerChoiceSharedData.h
        prj/PartnerChoice/src/PartnerChoiceOpportunity.cpp
        prj/PartnerChoice/include/PartnerChoiceOpportunity.h
        prj/PartnerChoice/src/PartnerChoiceWorldModel.cpp
        prj/PartnerChoice/include/PartnerChoiceWorldModel.h
        src/ext/PartnerChoiceConfigurationLoader.cpp
        include/ext/Config/PartnerChoiceConfigurationLoader.h

        # Partner control

        prj/PartnerChoice/src/PartnerChoiceAnalysisWorldObserver.cpp
        prj/PartnerChoice/include/PartnerChoiceAnalysisWorldObserver.h
        prj/PartnerControl/include/PartnerControlAgentObserver.h
        prj/PartnerControl/include/PartnerControlAnalysisWorldObserver.h
        prj/PartnerControl/include/PartnerControlController.h
        prj/PartnerControl/include/PartnerControlOpportunity.h
        prj/PartnerControl/include/PartnerControlWorldModel.h
        prj/PartnerControl/include/PartnerControlSharedData.h
        prj/PartnerControl/include/PartnerControlWorldObserver.h
        prj/PartnerControl/src/PartnerControlAgentObserver.cpp
        prj/PartnerControl/src/PartnerControlAnalysisWorldObserver.cpp
        prj/PartnerControl/src/PartnerControlController.cpp
        prj/PartnerControl/src/PartnerControlOpportunity.cpp
        prj/PartnerControl/src/PartnerControlSharedData.cpp
        prj/PartnerControl/src/PartnerControlWorldModel.cpp
        prj/PartnerControl/src/PartnerControlWorldObserver.cpp
        src/ext/PartnerControlConfigurationLoader.cpp
        include/ext/Config/PartnerControlConfigurationLoader.h
        src/contrib/network/NetworkInterface.cpp include/contrib/network/NetworkInterface.h

        # Fast Wanderer

        prj/FastWanderer/include/FastWandererAgentObserver.h
        prj/FastWanderer/include/FastWandererController.h
        prj/FastWanderer/include/FastWandererOpportunity.h
        prj/FastWanderer/include/FastWandererSharedData.h
        prj/FastWanderer/include/FastWandererWorldObserver.h

        prj/FastWanderer/src/FastWandererAgentObserver.cpp
        prj/FastWanderer/src/FastWandererController.cpp
        prj/FastWanderer/src/FastWandererSharedData.cpp
        prj/FastWanderer/src/FastWandererWorldObserver.cpp
        include/ext/Config/FastWandererConfigurationLoader.h
        src/ext/FastWandererConfigurationLoader.cpp
        src/contrib/network/PyevoInterface.cpp
        include/contrib/network/PyevoInterface.h prj/CoopFixed2/src/CoopFixed2AnalysisWorldObserver.cpp prj/CoopFixed2/include/CoopFixed2AnalysisWorldObserver.h prj/CoopFixed2/src/CoopFixed2AnalysisOpportunity.cpp prj/CoopFixed2/include/CoopFixed2AnalysisOpportunity.h
        prj/CorrectRepartition/include/CorrectRepartitionAgentObserver.h
        prj/CorrectRepartition/include/CorrectRepartitionAnalysisWorldObserver.h
        prj/CorrectRepartition/include/CorrectRepartitionController.h
        prj/CorrectRepartition/include/CorrectRepartitionOpportunity.h
        prj/CorrectRepartition/include/CorrectRepartitionSharedData.h
        prj/CorrectRepartition/include/CorrectRepartitionWorldModel.h
        prj/CorrectRepartition/include/CorrectRepartitionWorldObserver.h
        prj/CorrectRepartition/src/CorrectRepartitionAgentObserver.cpp
        prj/CorrectRepartition/src/CorrectRepartitionAnalysisWorldObserver.cpp
        prj/CorrectRepartition/src/CorrectRepartitionController.cpp
        prj/CorrectRepartition/src/CorrectRepartitionOpportunity.cpp
        prj/CorrectRepartition/src/CorrectRepartitionSharedData.cpp
        prj/CorrectRepartition/src/CorrectRepartitionWorldModel.cpp
        prj/CorrectRepartition/src/CorrectRepartitionWorldObserver.cpp
        src/ext/CorrectRepartitionConfigurationLoader.cpp

        src/contrib/neuralnetworks/EigenMLP.cpp
        include/contrib/neuralnetworks/EigenMLP.h

        # debug collision issue
        prj/DebugColl/include/DebugCollAgentObserver.h
        prj/DebugColl/include/DebugCollController.h
        prj/DebugColl/include/DebugCollWorldObserver.h
        prj/DebugColl/src/DebugCollAgentObserver.cpp
        prj/DebugColl/src/DebugCollController.cpp
        prj/DebugColl/src/DebugCollWorldObserver.cpp
        src/ext/DebugCollConfigurationLoader.cpp
        prj/DebugColl/src/DebugCollObj.cpp prj/DebugColl/include/DebugCollObj.h)

include_directories(include)
include_directories(include/contrib)
include_directories(include/core)
include_directories(include/ext)
include_directories(prj)


set(Boost_USE_STATIC_LIBS   ON)
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")  # use static libstdc++ which is not buggy
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# SDL IMPORT
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")


# Add git info as define

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(Git)


# the commit's SHA1, and whether the building workspace was dirty or not
execute_process(COMMAND
        "${GIT_EXECUTABLE}" describe --match=NeVeRmAtCh --always --abbrev=40 --dirty
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_SHA1
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the date of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%ad --date=local
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the subject of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%s
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_SUBJECT
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# generate version.cc
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.cpp.in" "${CMAKE_CURRENT_BINARY_DIR}/version.cpp" @ONLY)

list(APPEND SOURCE_FILES "${CMAKE_CURRENT_BINARY_DIR}/version.cpp" include/version.h)


find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(TBB REQUIRED)
find_package(Boost COMPONENTS filesystem system REQUIRED)


include_directories(${SDL2_INCLUDE_DIRS} ${SDL2_IMAGE_INCLUDE_DIRS})

add_executable(roborobo ${SOURCE_FILES})

target_link_libraries(roborobo
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        Eigen3::Eigen
       # tbb
        )
