#if defined PRJ_DEBUGCOLL || !defined MODULAR

#include "Config/DebugCollConfigurationLoader.h"
#include "DebugColl/include/DebugCollWorldObserver.h"
#include "DebugColl/include/DebugCollAgentObserver.h"
#include "DebugColl/include/DebugCollController.h"
#include "WorldModels/RobotWorldModel.h"


DebugCollConfigurationLoader::DebugCollConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

DebugCollConfigurationLoader::~DebugCollConfigurationLoader()
{
	//nothing to do
}

WorldObserver* DebugCollConfigurationLoader::make_WorldObserver(World* wm)
{
	return new DebugCollWorldObserver(wm);
}

RobotWorldModel* DebugCollConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* DebugCollConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new DebugCollAgentObserver(wm);
}

Controller* DebugCollConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new DebugCollController(wm);
}


#endif
