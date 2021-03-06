//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_LIONCONFIGURATIONLOADER_H
#define ROBOROBO3_LIONCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class LionConfigurationLoader : public ConfigurationLoader
{
public:
    LionConfigurationLoader() = default;

    ~LionConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World *wm);

    RobotWorldModel *make_RobotWorldModel();

    AgentObserver *make_AgentObserver(RobotWorldModel *wm);

    Controller *make_Controller(RobotWorldModel *wm);
};


#endif //ROBOROBO3_LIONCONFIGURATIONLOADER_H