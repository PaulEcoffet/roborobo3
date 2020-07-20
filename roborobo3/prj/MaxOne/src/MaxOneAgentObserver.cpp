//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "MaxOne/include/MaxOneAgentObserver.h"
#include <SDL2/SDL.h>

MaxOneAgentObserver::MaxOneAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<MaxOneWorldModel *>(wm);
    _wm = wm;
}

MaxOneAgentObserver::~MaxOneAgentObserver() = default;


void MaxOneAgentObserver::stepPre()
{
}

void MaxOneAgentObserver::stepPost()
{

}

void MaxOneAgentObserver::reset()
{
    AgentObserver::reset();
}
