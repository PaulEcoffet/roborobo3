//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <Negociate/include/NegociateAnalysisWorldObserver.h>
#include "Negociate/include/NegociateWorldObserver.h"
#include "Negociate/include/NegociateWorldModel.h"
#include "Negociate/include/NegociateAgentObserver.h"
#include "Negociate/include/NegociateController.h"
#include "Config/NegociateConfigurationLoader.h"

WorldObserver *NegociateConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    if (analysis)
    {
        return new NegociateAnalysisWorldObserver(wm);
    }
    else
    {
        return new NegociateWorldObserver(wm);
    }
}

RobotWorldModel *NegociateConfigurationLoader::make_RobotWorldModel()
{
    return new NegociateWorldModel();
}

AgentObserver *NegociateConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new NegociateAgentObserver(wm);
}

Controller *NegociateConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new NegociateController(wm);
}



