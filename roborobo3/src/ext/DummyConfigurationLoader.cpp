#if defined PRJ_DUMMY || !defined MODULAR

#include "Config/DummyConfigurationLoader.h"
#include "Dummy/include/DummyWorldObserver.h"
#include "Dummy/include/DummyAgentObserver.h"
#include "Dummy/include/DummyController.h"
#include "WorldModels/RobotWorldModel.h"

DummyConfigurationLoader::DummyConfigurationLoader()
{
}

DummyConfigurationLoader::~DummyConfigurationLoader()
{
	//nothing to do
}

WorldObserver* DummyConfigurationLoader::make_WorldObserver(World* wm)
{
	return new DummyWorldObserver(wm);
}

RobotWorldModel* DummyConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* DummyConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new DummyAgentObserver(wm);
}

Controller* DummyConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new DummyController(wm);
}

#endif
