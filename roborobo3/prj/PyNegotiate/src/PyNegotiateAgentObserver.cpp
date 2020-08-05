//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "PyNegotiate/include/PyNegotiateAgentObserver.h"
#include "PyNegotiate/include/PyNegotiateSharedData.h"
#include <SDL2/SDL.h>

PyNegotiateAgentObserver::PyNegotiateAgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<PyNegotiateWorldModel *>(wm);
    _wm = wm;
    m_seekTime = 0;
}

PyNegotiateAgentObserver::~PyNegotiateAgentObserver() = default;


void PyNegotiateAgentObserver::stepPre()
{
    if (!m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

int PyNegotiateAgentObserver::getSeekTime() const
{
    return m_seekTime;
}

void PyNegotiateAgentObserver::stepPost()
{
    color_agent();

    if (m_wm->seeking)
    {
        m_seekTime++;
        mark_walking_opp();
    }
}

void PyNegotiateAgentObserver::mark_walking_opp() const
{
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

void PyNegotiateAgentObserver::color_agent() const
{
    if (not m_wm->seeking)
    {
        m_wm->setRobotLED_colorValues(220, 220, 220);
    }
    else if (m_wm->fakeCoef < -0.33 * PyNegotiateSharedData::fakeCoef)
    {
        m_wm->setRobotLED_colorValues(126, 55, 49);
    }
    else if (m_wm->fakeCoef < +0.33 * PyNegotiateSharedData::fakeCoef)
    {
        m_wm->setRobotLED_colorValues(0, 0, 255);
    }
    else
    {
        m_wm->setRobotLED_colorValues(115, 182, 234);
    }
}

void PyNegotiateAgentObserver::reset()
{
    AgentObserver::reset();
    m_seekTime = 0;
}
