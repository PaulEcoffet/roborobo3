/*
 * CoopFixed2ConfigurationLoader.h
 */

#ifndef COOPFIXED2CONFIGURATIONLOADER_H
#define COOPFIXED2CONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class CoopFixed2ConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		CoopFixed2ConfigurationLoader();
		~CoopFixed2ConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
