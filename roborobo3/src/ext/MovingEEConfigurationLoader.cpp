#if defined PRJ_MOVINGEE || !defined MODULAR

#include "Config/MovingEEConfigurationLoader.h"

#include "MovingEE/include/MovingEEWorldObserver.h"
#include "MovingEE/include/MovingEEAgentObserver.h"
#include "MovingEE/include/MovingEEController.h"

#include "WorldModels/RobotWorldModel.h"

MovingEEConfigurationLoader::MovingEEConfigurationLoader()
{
}

MovingEEConfigurationLoader::~MovingEEConfigurationLoader()
{
	//nothing to do
}

WorldObserver* MovingEEConfigurationLoader::make_WorldObserver(World* wm)
{
	return new MovingEEWorldObserver(wm);
}

RobotWorldModel* MovingEEConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* MovingEEConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new MovingEEAgentObserver(wm);
}

Controller* MovingEEConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new MovingEEController(wm);
}

#endif
