//
// Created by paul on 02/11/17.
//

#include <core/RoboroboMain/main.h>
#include <PartnerChoice/include/PartnerChoiceAnalysisWorldObserver.h>
#include "PartnerChoice/include/PartnerChoiceWorldObserver.h"
#include "PartnerChoice/include/PartnerChoiceWorldModel.h"
#include "PartnerChoice/include/PartnerChoiceAgentObserver.h"
#include "PartnerChoice/include/PartnerChoiceController.h"
#include "ext/Config/PartnerChoiceConfigurationLoader.h"

WorldObserver *PartnerChoiceConfigurationLoader::make_WorldObserver(World *wm)
{
    bool analysis = false;
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    if (analysis)
    {
        return new PartnerChoiceAnalysisWorldObserver(wm);
    }
    else
    {
        return new PartnerChoiceWorldObserver(wm);
    }
}

RobotWorldModel *PartnerChoiceConfigurationLoader::make_RobotWorldModel()
{
    return new PartnerChoiceWorldModel();
}

AgentObserver *PartnerChoiceConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new PartnerChoiceAgentObserver(wm);
}

Controller *PartnerChoiceConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new PartnerChoiceController(wm);
}



