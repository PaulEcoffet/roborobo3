//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_PARTNERCHOICECONFIGURATIONLOADER_H
#define ROBOROBO3_PARTNERCHOICECONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class PartnerChoiceConfigurationLoader : public ConfigurationLoader
{
public:
    PartnerChoiceConfigurationLoader() = default;
    ~PartnerChoiceConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_PARTNERCHOICECONFIGURATIONLOADER_H