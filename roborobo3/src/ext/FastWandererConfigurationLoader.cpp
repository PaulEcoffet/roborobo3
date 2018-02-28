//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include "FastWanderer/include/FastWandererWorldObserver.h"
#include "FastWanderer/include/FastWandererAgentObserver.h"
#include "ext/Config/FastWandererConfigurationLoader.h"
#include "WorldModels/RobotWorldModel.h"

WorldObserver *FastWandererConfigurationLoader::make_WorldObserver(World *wm)
{
    return new FastWandererWorldObserver(wm);
}

RobotWorldModel *FastWandererConfigurationLoader::make_RobotWorldModel()
{
    return new RobotWorldModel();
}

AgentObserver *FastWandererConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new FastWandererAgentObserver(wm);
}

Controller *FastWandererConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new FastWandererController(wm);
}



