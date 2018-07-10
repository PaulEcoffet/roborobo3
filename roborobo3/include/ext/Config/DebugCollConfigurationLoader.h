/*
 * DebugCollConfigurationLoader.h
 */

#ifndef DEBUGCOLLCONFIGURATIONLOADER_H
#define	DEBUGCOLLCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class DebugCollConfigurationLoader : public ConfigurationLoader
{
	public:
		DebugCollConfigurationLoader();
		~DebugCollConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
