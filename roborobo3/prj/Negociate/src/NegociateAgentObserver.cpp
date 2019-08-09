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
    int dest_obj = -1;
    if (!m_wm->toBeTeleported && m_wm->teleport && !(NegociateSharedData::partnerControl && m_wm->nbOnOpp >= 2 && m_wm->arrival <= 2))
    {
        auto rob = gWorld->getRobot(this->m_wm->getId());
        rob->unregisterRobot();
        rob->setCoord(0,0);
        rob->setCoordReal(0,0);
        m_wm->toBeTeleported = true;
    }
    if (m_wm->toBeTeleported && random() < NegociateSharedData::tpProba)
    {
        m_wm->toBeTeleported = false;
        double angle = (float) this->m_wm->getId() / gNbOfRobots * 2 * M_PI;
        if (NegociateSharedData::smartTeleport)
        {
            std::vector<int> obj_shuffle(gNbOfPhysicalObjects);
            for (int i = 0; i < gNbOfPhysicalObjects; i++)
            {
                obj_shuffle[i] = i;
            }
            std::shuffle(obj_shuffle.begin(), obj_shuffle.end(), engine);
            for (int i = 0; i < gNbOfPhysicalObjects; i++)
            {
                int cur_obj = obj_shuffle[i];
                auto *cur = dynamic_cast<NegociateOpportunity *>(gPhysicalObjects[cur_obj]);
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
        else if (NegociateSharedData::nbCluster > 1 && m_wm->prevopp != -1)
        {
            const int nbCluster = NegociateSharedData::nbCluster;
            const int objpercluster = gNbOfPhysicalObjects / nbCluster;
            int cur_cluster = m_wm->prevopp / objpercluster;
            double p_stayincluster = (NegociateSharedData::pStayInCluster != -1)? NegociateSharedData::pStayInCluster : 1 - (1.0 / (gNbOfPhysicalObjects / nbCluster));
            if (random() < p_stayincluster) // Stay in the same cluster
            {
                int curloc = m_wm->prevopp % objpercluster;
                std::uniform_int_distribution<int> dis(0, objpercluster-1 -1);
                int newloc = dis(engine);
                if (newloc >= curloc) newloc += 1; // exclude the current location from the draw
                dest_obj = cur_cluster * objpercluster + newloc;
            }
            else
            {
                std::uniform_int_distribution<int> dis(0, nbCluster-1 -1);
                std::uniform_int_distribution<int> disInCluster(0, objpercluster-1);
                int newclus = dis(engine);
                if (newclus >= cur_cluster) newclus += 1; // exclude the current location from the draw
                dest_obj = newclus * objpercluster + disInCluster(engine);
            }
        }
        else if (NegociateSharedData::proximityTeleport != 0 && m_wm->prevopp != -1)
        {
            std::uniform_int_distribution<int> dis(1, NegociateSharedData::proximityTeleport);
            std::uniform_int_distribution<int> signdis(0, 1);
            int sign = signdis(engine) * 2 - 1;
            dest_obj = (gNbOfPhysicalObjects + m_wm->prevopp + sign * dis(engine)) % gNbOfPhysicalObjects;
        }
        else
        {
            std::uniform_int_distribution<int> dis(0, gNbOfPhysicalObjects-1);
            dest_obj = dis(engine);
        }
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
    }

    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage, static_cast<int>(_wm->_xReal + 0.5),
                              static_cast<int>(_wm->_yReal + 0.5));
    SDL_GetRGB(pixel, gFootprintImage->format, &r, &g, &b);
    _wm->_groundSensorValue[0] = (int)r;
    _wm->_groundSensorValue[1] = (int)g;
    _wm->_groundSensorValue[2] = (int)b;
    int targetIndex = _wm->getGroundSensorValue();
    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset);
        m_wm->prevopp = targetIndex;
        if (m_wm->teleport && dest_obj != -1 && targetIndex != dest_obj)
        {
            std::cerr << "Not on opp for tp : " << m_wm->getId() << " :" << targetIndex << " " << dest_obj << "\n";
            m_wm->setRobotLED_colorValues(0, 0, 0);
            //exit(1);
        }
    }
}

void NegociateAgentObserver::reset()
{
    AgentObserver::reset();
}
