//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H
#define ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class PyNegotiateConfigurationLoader : public ConfigurationLoader
{
public:
    PyNegotiateConfigurationLoader() = default;

    ~PyNegotiateConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World *wm);

    RobotWorldModel *make_RobotWorldModel();

    AgentObserver *make_AgentObserver(RobotWorldModel *wm);

    Controller *make_Controller(RobotWorldModel *wm);
};


#endif //ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H