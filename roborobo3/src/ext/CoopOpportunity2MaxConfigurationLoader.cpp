#if defined PRJ_COOPOPPORTUNITY2MAX || !defined MODULAR

#include "Config/CoopOpportunity2MaxConfigurationLoader.h"

#include "CoopOpportunity2Max/include/CoopOpportunity2MaxWorldObserver.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxAgentObserver.h"
#include "CoopOpportunity2Max/include/CoopOpportunity2MaxController.h"

#include "WorldModels/RobotWorldModel.h"

CoopOpportunity2MaxConfigurationLoader::CoopOpportunity2MaxConfigurationLoader()
{
}

CoopOpportunity2MaxConfigurationLoader::~CoopOpportunity2MaxConfigurationLoader()
{
	//nothing to do
}

WorldObserver* CoopOpportunity2MaxConfigurationLoader::make_WorldObserver(World* wm)
{
	return new CoopOpportunity2MaxWorldObserver(wm);
}

RobotWorldModel* CoopOpportunity2MaxConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* CoopOpportunity2MaxConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new CoopOpportunity2MaxAgentObserver(wm);
}

Controller* CoopOpportunity2MaxConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new CoopOpportunity2MaxController(wm);
}

#endif
