//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include "pyFastWanderer/include/pyFastWandererWorldObserver.h"
#include "pyFastWanderer/include/pyFastWandererAgentObserver.h"
#include "Config/pyFastWandererConfigurationLoader.h"
#include "WorldModels/RobotWorldModel.h"

WorldObserver *pyFastWandererConfigurationLoader::make_WorldObserver(World *wm)
{
    return new pyFastWandererWorldObserver(wm);
}

RobotWorldModel *pyFastWandererConfigurationLoader::make_RobotWorldModel()
{
    return new RobotWorldModel();
}

AgentObserver *pyFastWandererConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new pyFastWandererAgentObserver(wm);
}

Controller *pyFastWandererConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new pyFastWandererController(wm);
}



