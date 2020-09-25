/*
 *	ConfigurationLoader.h
 */

#ifndef CONFIGURATIONLOADER_H
#define CONFIGURATIONLOADER_H

#include <core/World/PhysicalObject.h>
#include "RoboroboMain/common.h"

class WorldObserver;
class RobotWorldModel;
class AgentObserver;
class Controller;
class World;

class ConfigurationLoader
{
protected:
    ConfigurationLoader() = default;

public:
    virtual ~ConfigurationLoader() = default;

    static ConfigurationLoader *make_ConfigurationLoader(const std::string &configurationLoaderObjectName);

    virtual WorldObserver *make_WorldObserver(World *wm) = 0;

    virtual RobotWorldModel *make_RobotWorldModel() = 0;

    virtual AgentObserver *make_AgentObserver(RobotWorldModel *wm) = 0;

    virtual Controller *make_Controller(RobotWorldModel *wm) = 0;

    virtual PhysicalObject *make_CustomObject(int id);
};


#endif
