//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_FASTWANDERERCONFIGURATIONLOADER_H
#define ROBOROBO3_FASTWANDERERCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class FastWandererConfigurationLoader : public ConfigurationLoader
{
public:
    FastWandererConfigurationLoader() = default;
    ~FastWandererConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_FASTWANDERERCONFIGURATIONLOADER_H