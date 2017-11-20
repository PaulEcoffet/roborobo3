//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_PARTNERCONTROLCONFIGURATIONLOADER_H
#define ROBOROBO3_PARTNERCONTROLCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class PartnerControlConfigurationLoader : public ConfigurationLoader
{
public:
    PartnerControlConfigurationLoader() = default;
    ~PartnerControlConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_PARTNERCONTROLCONFIGURATIONLOADER_H