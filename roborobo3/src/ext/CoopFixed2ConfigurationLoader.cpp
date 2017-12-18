//
// Created by paul on 02/11/17.
//

#include <core/RoboroboMain/main.h>
#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2WorldModel.h"
#include "CoopFixed2/include/CoopFixed2AgentObserver.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "ext/Config/CoopFixed2ConfigurationLoader.h"

WorldObserver *CoopFixed2ConfigurationLoader::make_WorldObserver(World *wm)
{
    return new CoopFixed2WorldObserver(wm);
}

RobotWorldModel *CoopFixed2ConfigurationLoader::make_RobotWorldModel()
{
    return new CoopFixed2WorldModel();
}

AgentObserver *CoopFixed2ConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new CoopFixed2AgentObserver(wm);
}

Controller *CoopFixed2ConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new CoopFixed2Controller(wm);
}



