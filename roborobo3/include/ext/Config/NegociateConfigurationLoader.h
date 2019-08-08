//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_NEGOCIATECONFIGURATIONLOADER_H
#define ROBOROBO3_NEGOCIATECONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class NegociateConfigurationLoader : public ConfigurationLoader
{
public:
    NegociateConfigurationLoader() = default;
    ~NegociateConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_NEGOCIATECONFIGURATIONLOADER_H