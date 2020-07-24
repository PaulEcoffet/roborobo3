/*
 * MedeaConfigurationLoader.h
 */

#ifndef DUMMYCONFIGURATIONLOADER_H
#define DUMMYCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"


class DummyConfigurationLoader : public ConfigurationLoader
{
private:

public:
    DummyConfigurationLoader();
    ~DummyConfigurationLoader();

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif