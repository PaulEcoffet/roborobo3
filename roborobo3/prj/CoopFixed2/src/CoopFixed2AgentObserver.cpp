//
// Created by paul on 30/10/17.
//

#include <core/Utilities/Graphics.h>
#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "CoopFixed2/include/CoopFixed2AgentObserver.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"
#include <SDL2/SDL.h>

CoopFixed2AgentObserver::CoopFixed2AgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<CoopFixed2WorldModel *>(wm);
    _wm = wm;
}

CoopFixed2AgentObserver::~CoopFixed2AgentObserver() = default;


void CoopFixed2AgentObserver::stepPre()
{
    if (!m_wm->onOpportunity)
    {
        m_wm->lastOwnInvest.clear();
        m_wm->lastTotalInvest.clear();
    }
}

void CoopFixed2AgentObserver::stepPost()
{
    int dest_obj = -1;
    std::vector<int> obj_shuffle(gNbOfPhysicalObjects);
    for (int i = 0; i < gNbOfPhysicalObjects; i++)
    {
        obj_shuffle[i] = i;
    }
    std::shuffle(obj_shuffle.begin(), obj_shuffle.end(), engine);
    if (m_wm->teleport && !(CoopFixed2SharedData::partnerControl && m_wm->nbOnOpp >= 2 && m_wm->arrival <= 2))
    {
        //std::cout << CoopFixed2SharedData::partnerControl << " " << m_wm->nbOnOpp << " " << ((m_wm->opp)? m_wm->opp->getNbNearbyRobots(): 0) << std::endl;
        double angle = (float) this->m_wm->getId() / gNbOfRobots * 2 * M_PI;
        if (CoopFixed2SharedData::smartTeleport)
        {
            for (int i = 0; i < gNbOfPhysicalObjects; i++)
            {
                int cur_obj = obj_shuffle[i];
                auto *cur = dynamic_cast<CoopFixed2Opportunity *>(gPhysicalObjects[cur_obj]);
                //std::cout << "rob " << this->m_wm->getId() << " " << i <<" : " << cur->getNbNearbyRobots() << " " << cur->getNbNewNearbyRobots() << "\n";
                if (cur->getNbNearbyRobots() + cur->getNbNewNearbyRobots() == 1)
                {
                    dest_obj = cur_obj;
                    break;
                }
                else if (cur->getNbNearbyRobots() + cur->getNbNewNearbyRobots() == 0 && dest_obj == -1)
                {
                    dest_obj = cur_obj;
                }
            }
        }
        else
        {
            dest_obj = obj_shuffle[0];
        }
        if (dest_obj != -1)
        {
            PhysicalObject *physobj = gPhysicalObjects[dest_obj];
            auto rob = gWorld->getRobot(this->m_wm->getId());
            rob->unregisterRobot();
            rob->setCoord(
                    static_cast<int>(physobj->getXCenterPixel() +
                                     cos(angle) * (1 + gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                    static_cast<int>(physobj->getYCenterPixel() +
                                     sin(angle) * (1 + gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
            rob->setCoordReal(
                    static_cast<int>(physobj->getXCenterPixel() +
                                     cos(angle) * (1 + gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                    static_cast<int>(physobj->getYCenterPixel() +
                                     sin(angle) * (1 + gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
            rob->getWorldModel()->_agentAbsoluteOrientation = 0;
            rob->registerRobot();
        }
    }

    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage, static_cast<int>(_wm->_xReal + 0.5),
                              static_cast<int>(_wm->_yReal + 0.5));
    SDL_GetRGB(pixel, gFootprintImage->format, &r, &g, &b);
    _wm->_groundSensorValue[0] = r;
    _wm->_groundSensorValue[1] = g;
    _wm->_groundSensorValue[2] = b;
    int targetIndex = m_wm->getGroundSensorValue();
    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset);
        if (m_wm->teleport && dest_obj != -1 && targetIndex != dest_obj)
        {
            std::cerr << "ERROR: " << targetIndex << " " << dest_obj << "\n";
            m_wm->setRobotLED_colorValues(0, 0, 0);
            //exit(1);
        }
    }
}

void CoopFixed2AgentObserver::reset()
{
    AgentObserver::reset();
}
