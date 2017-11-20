//
// Created by paul on 02/11/17.
//

#include "PartnerControl/include/PartnerControlWorldObserver.h"
#include "PartnerControl/include/PartnerControlWorldModel.h"
#include "PartnerControl/include/PartnerControlAgentObserver.h"
#include "PartnerControl/include/PartnerControlController.h"
#include "ext/Config/PartnerControlConfigurationLoader.h"

WorldObserver *PartnerControlConfigurationLoader::make_WorldObserver(World *wm)
{
    return new PartnerControlWorldObserver(wm);
}

RobotWorldModel *PartnerControlConfigurationLoader::make_RobotWorldModel()
{
    return new PartnerControlWorldModel();
}

AgentObserver *PartnerControlConfigurationLoader::make_AgentObserver(RobotWorldModel *wm)
{
    return new PartnerControlAgentObserver(wm);
}

Controller *PartnerControlConfigurationLoader::make_Controller(RobotWorldModel *wm)
{
    return new PartnerControlController(wm);
}



