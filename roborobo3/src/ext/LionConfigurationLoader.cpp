//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <Lion/include/LionAnalysisWorldObserver.h>
#include <Lion/include/LionTournamentWorldObserver.h>
#include "Lion/include/LionWorldObserver.h"
#include "Lion/include/LionWorldModel.h"
#include "Lion/include/LionAgentObserver.h"
#include "Lion/include/LionController.h"
#include "Config/LionConfigurationLoader.h"

WorldObserver *LionConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    bool tournament = false;
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    gProperties.checkAndGetPropertyValue("tournament", &tournament, false);
    if (tournament)
    {
        return new LionTournamentWorldObserver(wm);
    }
    if (analysis)
    {
        return new LionAnalysisWorldObserver(wm);
    }
    else
    {
        return new LionWorldObserver(wm);
    }
}

RobotWorldModel *LionConfigurationLoader::make_RobotWorldModel()
{
    return new LionWorldModel();
}

AgentObserver *LionConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new LionAgentObserver(wm);
}

Controller *LionConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new LionController(wm);
}



