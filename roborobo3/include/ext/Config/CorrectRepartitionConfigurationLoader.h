//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_CORRECTREPARTITIONCONFIGURATIONLOADER_H
#define ROBOROBO3_CORRECTREPARTITIONCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class CorrectRepartitionConfigurationLoader : public ConfigurationLoader
{
public:
    CorrectRepartitionConfigurationLoader() = default;
    ~CorrectRepartitionConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_CORRECTREPARTITIONCONFIGURATIONLOADER_H