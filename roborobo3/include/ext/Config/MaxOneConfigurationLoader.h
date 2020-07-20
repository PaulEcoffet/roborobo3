//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_MAXONECONFIGURATIONLOADER_H
#define ROBOROBO3_MAXONECONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class MaxOneConfigurationLoader : public ConfigurationLoader
{
public:
    MaxOneConfigurationLoader() = default;
    ~MaxOneConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_MAXONECONFIGURATIONLOADER_H