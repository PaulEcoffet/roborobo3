//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <NegociateGym/include/NegociateGymAnalysisWorldObserver.h>
#include <NegociateGym/include/NegociateGymTrainWorldObserver.h>

#include "NegociateGym/include/NegociateGymWorldObserver.h"
#include "NegociateGym/include/NegociateGymWorldModel.h"
#include "NegociateGym/include/NegociateGymAgentObserver.h"
#include "NegociateGym/include/NegociateGymController.h"
#include "Config/NegociateGymConfigurationLoader.h"

WorldObserver *NegociateGymConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    bool train = false;
    gProperties.checkAndGetPropertyValue("train", &train, false);
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    if (train)
    {
        return new NegociateGymTrainWorldObserver(wm);
    }
    else if (analysis)
    {
        return new NegociateGymAnalysisWorldObserver(wm);
    }
    else
    {
        return new NegociateGymWorldObserver(wm);
    }
}

RobotWorldModel *NegociateGymConfigurationLoader::make_RobotWorldModel()
{
    return new NegociateGymWorldModel();
}

AgentObserver *NegociateGymConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new NegociateGymAgentObserver(wm);
}

Controller *NegociateGymConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new NegociateGymController(wm);
}



