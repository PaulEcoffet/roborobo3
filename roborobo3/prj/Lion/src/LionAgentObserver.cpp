//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "Lion/include/LionAgentObserver.h"
#include "Lion/include/LionSharedData.h"
#include "Lion/include/LionWorldObserver.h"
#include <SDL2/SDL.h>

LionAgentObserver::LionAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<LionWorldModel *>(wm);
    _wm = wm;
}

LionAgentObserver::~LionAgentObserver() = default;


void LionAgentObserver::stepPre()
{

}

void LionAgentObserver::stepPost()
{

}

void LionAgentObserver::reset()
{
    AgentObserver::reset();
}
