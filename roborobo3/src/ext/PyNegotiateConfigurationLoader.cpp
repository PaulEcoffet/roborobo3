//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <PyNegotiate/include/PyNegotiateAnalysisWorldObserver.h>
#include <PyNegotiate/include/PyNegotiateTrainWorldObserver.h>

#include "PyNegotiate/include/PyNegotiateWorldObserver.h"
#include "PyNegotiate/include/PyNegotiateWorldModel.h"
#include "PyNegotiate/include/PyNegotiateAgentObserver.h"
#include "PyNegotiate/include/PyNegotiateController.h"
#include "Config/PyNegotiateConfigurationLoader.h"

WorldObserver *PyNegotiateConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    bool train = false;
    gProperties.checkAndGetPropertyValue("train", &train, false);
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    if (train)
    {
        return new PyNegotiateTrainWorldObserver(wm);
    }
    else if (analysis)
    {
        return new PyNegotiateAnalysisWorldObserver(wm);
    }
    else
    {
        return new PyNegotiateWorldObserver(wm);
    }
}

RobotWorldModel *PyNegotiateConfigurationLoader::make_RobotWorldModel()
{
    return new PyNegotiateWorldModel();
}

AgentObserver *PyNegotiateConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new PyNegotiateAgentObserver(wm);
}

Controller *PyNegotiateConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new PyNegotiateController(wm);
}



