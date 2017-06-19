/*
 * SingleGenomeConfigurationLoader.h
 */

#ifndef SINGLEGENOMECONFIGURATIONLOADER_H
#define SINGLEGENOMECONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class SingleGenomeConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		SingleGenomeConfigurationLoader();
		~SingleGenomeConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
