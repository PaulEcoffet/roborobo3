/*
 * TemplateMovingConfigurationLoader.h
 */

#ifndef TEMPLATEMOVINGCONFIGURATIONLOADER_H
#define	TEMPLATEMOVINGCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class TemplateMovingConfigurationLoader : public ConfigurationLoader
{
	public:
		TemplateMovingConfigurationLoader();
		~TemplateMovingConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
