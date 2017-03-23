/*
 * MovingEEConfigurationLoader.h
 */

#ifndef MOVINGEECONFIGURATIONLOADER_H
#define MOVINGEECONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class MovingEEConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		MovingEEConfigurationLoader();
		~MovingEEConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
