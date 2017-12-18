//
// Created by paul on 02/11/17.
//

#ifndef ROBOROBO3_COOPFIXED2CONFIGURATIONLOADER_H
#define ROBOROBO3_COOPFIXED2CONFIGURATIONLOADER_H


#include "ConfigurationLoader.h"

class CoopFixed2ConfigurationLoader : public ConfigurationLoader
{
public:
    CoopFixed2ConfigurationLoader() = default;
    ~CoopFixed2ConfigurationLoader() = default;

    WorldObserver *make_WorldObserver(World* wm) ;
    RobotWorldModel *make_RobotWorldModel();
    AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
    Controller *make_Controller(RobotWorldModel* wm) ;
};


#endif //ROBOROBO3_COOPFIXED2CONFIGURATIONLOADER_H