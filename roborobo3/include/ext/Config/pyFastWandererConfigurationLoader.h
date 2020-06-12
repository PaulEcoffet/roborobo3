//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_PYFASTWANDERERCONFIGURATIONLOADER_H
#define ROBOROBO3_PYFASTWANDERERCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class pyFastWandererConfigurationLoader : public ConfigurationLoader
{
public:
    pyFastWandererConfigurationLoader() = default;
    ~pyFastWandererConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_PYFASTWANDERERCONFIGURATIONLOADER_H