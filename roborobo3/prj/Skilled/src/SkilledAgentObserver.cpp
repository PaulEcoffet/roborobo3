//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "Skilled/include/SkilledAgentObserver.h"
#include "Skilled/include/SkilledSharedData.h"
#include "Skilled/include/SkilledWorldObserver.h"
#include <SDL2/SDL.h>

SkilledAgentObserver::SkilledAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<SkilledWorldModel *>(wm);
    _wm = wm;
}

SkilledAgentObserver::~SkilledAgentObserver() = default;


void SkilledAgentObserver::stepPre()
{

}

void SkilledAgentObserver::stepPost()
{

}

void SkilledAgentObserver::reset()
{
    AgentObserver::reset();
}
