/*
 * CoopOpportunity2MaxConfigurationLoader.h
 */

#ifndef COOPOPPORTUNITY2MAXCONFIGURATIONLOADER_H
#define COOPOPPORTUNITY2MAXCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class CoopOpportunity2MaxConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		CoopOpportunity2MaxConfigurationLoader();
		~CoopOpportunity2MaxConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
