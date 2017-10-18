#if defined PRJ_COOPFIXED2 || !defined MODULAR

#include "Config/CoopFixed2ConfigurationLoader.h"

#include "CoopFixed2/include/CoopFixed2WorldObserver.h"
#include "CoopFixed2/include/CoopFixed2AgentObserver.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"

#include "WorldModels/RobotWorldModel.h"

CoopFixed2ConfigurationLoader::CoopFixed2ConfigurationLoader()
{
}

CoopFixed2ConfigurationLoader::~CoopFixed2ConfigurationLoader()
{
	//nothing to do
}

WorldObserver* CoopFixed2ConfigurationLoader::make_WorldObserver(World* wm)
{
	return new CoopFixed2WorldObserver(wm);
}

RobotWorldModel* CoopFixed2ConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* CoopFixed2ConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new CoopFixed2AgentObserver(wm);
}

Controller* CoopFixed2ConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new CoopFixed2Controller(wm);
}

#endif
