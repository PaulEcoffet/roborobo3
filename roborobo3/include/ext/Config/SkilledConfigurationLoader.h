//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_SKILLEDCONFIGURATIONLOADER_H
#define ROBOROBO3_SKILLEDCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class SkilledConfigurationLoader : public ConfigurationLoader {
public:
    SkilledConfigurationLoader() = default;

    ~SkilledConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World *wm);

    RobotWorldModel *make_RobotWorldModel();

    AgentObserver *make_AgentObserver(RobotWorldModel *wm);

    Controller *make_Controller(RobotWorldModel *wm);
};


#endif //ROBOROBO3_SKILLEDCONFIGURATIONLOADER_H