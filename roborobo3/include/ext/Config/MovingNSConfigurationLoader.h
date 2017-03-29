/*
 * MovingNSConfigurationLoader.h
 */

#ifndef MOVINGNSCONFIGURATIONLOADER_H
#define MOVINGNSCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class MovingNSConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		MovingNSConfigurationLoader();
		~MovingNSConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
