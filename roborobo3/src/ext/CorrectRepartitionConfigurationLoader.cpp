//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <CorrectRepartition/include/CorrectRepartitionAnalysisWorldObserver.h>
#include "CorrectRepartition/include/CorrectRepartitionWorldObserver.h"
#include "CorrectRepartition/include/CorrectRepartitionWorldModel.h"
#include "CorrectRepartition/include/CorrectRepartitionAgentObserver.h"
#include "CorrectRepartition/include/CorrectRepartitionController.h"
#include "Config/CorrectRepartitionConfigurationLoader.h"

WorldObserver *CorrectRepartitionConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    if (analysis)
    {
        return new CorrectRepartitionAnalysisWorldObserver(wm);
    }
    else
    {
        return new CorrectRepartitionWorldObserver(wm);
    }
}

RobotWorldModel *CorrectRepartitionConfigurationLoader::make_RobotWorldModel()
{
    return new CorrectRepartitionWorldModel();
}

AgentObserver *CorrectRepartitionConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new CorrectRepartitionAgentObserver(wm);
}

Controller *CorrectRepartitionConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new CorrectRepartitionController(wm);
}



