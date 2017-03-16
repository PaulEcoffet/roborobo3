#if defined PRJ_TEMPLATEMOVING || !defined MODULAR

#include "Config/TemplateMovingConfigurationLoader.h"

#include "TemplateMoving/include/TemplateMovingWorldObserver.h"
#include "TemplateMoving/include/TemplateMovingAgentObserver.h"
#include "TemplateMoving/include/TemplateMovingController.h"

#include "WorldModels/RobotWorldModel.h"


TemplateMovingConfigurationLoader::TemplateMovingConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

TemplateMovingConfigurationLoader::~TemplateMovingConfigurationLoader()
{
	//nothing to do
}

WorldObserver* TemplateMovingConfigurationLoader::make_WorldObserver(World* wm)
{
	return new TemplateMovingWorldObserver(wm);
}

RobotWorldModel* TemplateMovingConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* TemplateMovingConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new TemplateMovingAgentObserver(wm);
}

Controller* TemplateMovingConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new TemplateMovingController(wm);
}


#endif
