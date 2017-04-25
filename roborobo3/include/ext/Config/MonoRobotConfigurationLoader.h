/*
 * MonoRobotConfigurationLoader.h
 */

#ifndef MONOROBOTCONFIGURATIONLOADER_H
#define MONOROBOTCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class MonoRobotConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		MonoRobotConfigurationLoader();
		~MonoRobotConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
