//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_NEGOCIATEGYMCONFIGURATIONLOADER_H
#define ROBOROBO3_NEGOCIATEGYMCONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class NegociateGymConfigurationLoader : public ConfigurationLoader
{
public:
    NegociateGymConfigurationLoader() = default;
    ~NegociateGymConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_NEGOCIATEGYMCONFIGURATIONLOADER_H