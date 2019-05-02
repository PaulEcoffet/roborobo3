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
    int dest_obj = -1;
    dest_obj = m_wm->teleport;
    double angle = ((double)m_wm->getId() / gNbOfRobots) * 2 * M_PI;
    if (dest_obj != -1)
    {
        PhysicalObject *physobj = gPhysicalObjects[dest_obj];
        auto rob = gWorld->getRobot(this->m_wm->getId());
        rob->unregisterRobot();
        rob->setCoord(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
        rob->setCoordReal(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
        rob->getWorldModel()->_agentAbsoluteOrientation = 0;
        rob->registerRobot();
    }


    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage, static_cast<int>(m_wm->_xReal + 0.5),
                              static_cast<int>(m_wm->_yReal + 0.5));
    SDL_GetRGB(pixel, gFootprintImage->format, &r, &g, &b);
    m_wm->_groundSensorValue[0] = (int) r;
    m_wm->_groundSensorValue[1] = (int) g;
    m_wm->_groundSensorValue[2] = (int) b;


    int targetIndex = m_wm->getGroundSensorValue();
    bool newopp = false;

    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset); // callback on opportunity


        if (!m_wm->opp || targetIndex != m_wm->opp->getId())
        {
            newopp = true;
            if(m_wm->opp)
            {
                if (gVerbose)
                {
                    std::cout << m_wm->getId() << " moved from " << m_wm->opp->getId() << " to " << targetIndex << std::endl;
                }
                m_wm->opp->removeRobot(m_wm->getId());
            }
        }
        m_wm->opp = dynamic_cast<LionOpportunity*>(gPhysicalObjects[targetIndex]); // Agent is on this opp

        if (m_wm->teleport && dest_obj != -1 && targetIndex != dest_obj)
        {
            std::cerr << "Not on opp for tp : " << m_wm->getId() << " :" << targetIndex << " " << dest_obj << "\n";
            m_wm->setRobotLED_colorValues(0, 0, 0);
            //exit(1);
        }
    }


    double cost = (newopp)? 5 : 0;

    auto totalinv = m_wm->opp->getCurInv();
    int n = m_wm->opp->countCurrentRobots();
    double payoff = LionWorldObserver::payoff(m_wm->getCoop(n - 1), totalinv, n, LionSharedData::meanA, LionSharedData::b);

    m_wm->_fitnessValue += payoff - cost;

}

void LionAgentObserver::reset()
{
    AgentObserver::reset();
}
