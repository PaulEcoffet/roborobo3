#if defined PRJ_MOVINGNS || !defined MODULAR

#include "Config/MovingNSConfigurationLoader.h"

#include "MovingNS/include/MovingNSWorldObserver.h"
#include "MovingNS/include/MovingNSAgentObserver.h"
#include "MovingNS/include/MovingNSController.h"

#include "WorldModels/RobotWorldModel.h"

MovingNSConfigurationLoader::MovingNSConfigurationLoader()
{
}

MovingNSConfigurationLoader::~MovingNSConfigurationLoader()
{
	//nothing to do
}

WorldObserver* MovingNSConfigurationLoader::make_WorldObserver(World* wm)
{
	return new MovingNSWorldObserver(wm);
}

RobotWorldModel* MovingNSConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* MovingNSConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new MovingNSAgentObserver(wm);
}

Controller* MovingNSConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new MovingNSController(wm);
}

#endif
