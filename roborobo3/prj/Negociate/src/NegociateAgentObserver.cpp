//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "Negociate/include/NegociateAgentObserver.h"
#include "Negociate/include/NegociateSharedData.h"
#include <SDL2/SDL.h>

NegociateAgentObserver::NegociateAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<NegociateWorldModel *>(wm);
    _wm = wm;
}

NegociateAgentObserver::~NegociateAgentObserver() = default;


void NegociateAgentObserver::stepPre()
{
    if (!m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

void NegociateAgentObserver::stepPost()
{
    if (!m_wm->seeking)
    {
        return;
    }
    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage, static_cast<int>(_wm->_xReal + 0.5),
                              static_cast<int>(_wm->_yReal + 0.5));
    SDL_GetRGB(pixel, gFootprintImage->format, &r, &g, &b);
    _wm->_groundSensorValue[0] = (int) r;
    _wm->_groundSensorValue[1] = (int) g;
    _wm->_groundSensorValue[2] = (int) b;
    int targetIndex = _wm->getGroundSensorValue();
    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset);
        m_wm->prevopp = targetIndex;
    }
}

void NegociateAgentObserver::reset()
{
    AgentObserver::reset();
}
