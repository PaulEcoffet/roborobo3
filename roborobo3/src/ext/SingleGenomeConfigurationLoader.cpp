#if defined PRJ_SINGLEGENOME || !defined MODULAR

#include "Config/SingleGenomeConfigurationLoader.h"

#include "SingleGenome/include/SingleGenomeWorldObserver.h"
#include "SingleGenome/include/SingleGenomeAgentObserver.h"
#include "SingleGenome/include/SingleGenomeController.h"

#include "WorldModels/RobotWorldModel.h"

SingleGenomeConfigurationLoader::SingleGenomeConfigurationLoader()
{
}

SingleGenomeConfigurationLoader::~SingleGenomeConfigurationLoader()
{
	//nothing to do
}

WorldObserver* SingleGenomeConfigurationLoader::make_WorldObserver(World* wm)
{
	return new SingleGenomeWorldObserver(wm);
}

RobotWorldModel* SingleGenomeConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* SingleGenomeConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new SingleGenomeAgentObserver(wm);
}

Controller* SingleGenomeConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new SingleGenomeController(wm);
}

#endif
