//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>

#include "MaxOne/include/MaxOneWorldObserver.h"
#include "MaxOne/include/MaxOneWorldModel.h"
#include "MaxOne/include/MaxOneAgentObserver.h"
#include "MaxOne/include/MaxOneController.h"
#include "Config/MaxOneConfigurationLoader.h"

WorldObserver *MaxOneConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    bool train = false;
    gProperties.checkAndGetPropertyValue("train", &train, false);
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);

    return new MaxOneWorldObserver(wm);
}

RobotWorldModel *MaxOneConfigurationLoader::make_RobotWorldModel()
{
    return new MaxOneWorldModel();
}

AgentObserver *MaxOneConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new MaxOneAgentObserver(wm);
}

Controller *MaxOneConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new MaxOneController(wm);
}



