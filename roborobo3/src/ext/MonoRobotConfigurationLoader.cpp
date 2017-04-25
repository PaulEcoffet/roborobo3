#if defined PRJ_MONOROBOT || !defined MODULAR

#include "Config/MonoRobotConfigurationLoader.h"

#include "MonoRobot/include/MonoRobotWorldObserver.h"
#include "MonoRobot/include/MonoRobotAgentObserver.h"
#include "MonoRobot/include/MonoRobotController.h"

#include "WorldModels/RobotWorldModel.h"

MonoRobotConfigurationLoader::MonoRobotConfigurationLoader()
{
}

MonoRobotConfigurationLoader::~MonoRobotConfigurationLoader()
{
	//nothing to do
}

WorldObserver* MonoRobotConfigurationLoader::make_WorldObserver(World* wm)
{
	return new MonoRobotWorldObserver(wm);
}

RobotWorldModel* MonoRobotConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* MonoRobotConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new MonoRobotAgentObserver(wm);
}

Controller* MonoRobotConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new MonoRobotController(wm);
}

#endif
