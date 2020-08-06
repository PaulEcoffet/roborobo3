// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H
#define ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class PyNegotiateConfigurationLoader : public ConfigurationLoader
{
public:
    PyNegotiateConfigurationLoader() = default;

    ~PyNegotiateConfigurationLoader() override = default;

    WorldObserver *make_WorldObserver(World *wm) override;

    RobotWorldModel *make_RobotWorldModel() override;

    AgentObserver *make_AgentObserver(RobotWorldModel *wm) override;

    Controller *make_Controller(RobotWorldModel *wm) override;
};


#endif //ROBOROBO3_PYNEGOTIATECONFIGURATIONLOADER_H