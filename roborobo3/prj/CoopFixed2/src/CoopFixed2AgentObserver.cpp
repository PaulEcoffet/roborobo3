//
// Created by paul on 30/10/17.
//

#include "core/RoboroboMain/main.h"
#include "core/World/World.h"
#include "CoopFixed2/include/CoopFixed2AgentObserver.h"

CoopFixed2AgentObserver::CoopFixed2AgentObserver(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<CoopFixed2WorldModel*>(wm);
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

    if (m_wm->teleport)
    {
        auto randomPhys = std::uniform_int_distribution<int>(0, gNbOfPhysicalObjects - 1);
        int dest_obj = randomPhys(engine);
        double angle = (float) this->m_wm->getId() / gNbOfRobots * 2 * M_PI;
        PhysicalObject *physobj = gPhysicalObjects[dest_obj];
        auto rob = gWorld->getRobot(this->m_wm->getId());
        rob->unregisterRobot();
        rob->setCoord(physobj->getXCenterPixel()+ cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2),
                      physobj->getYCenterPixel()+ sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2));
        rob->setCoordReal(physobj->getXCenterPixel()+ cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2),
                          physobj->getYCenterPixel()+ sin(angle) *(gPhysicalObjectDefaultRadius + gRobotWidth / 2));
        rob->getWorldModel()->_agentAbsoluteOrientation = 0;
        rob->registerRobot();
    }


    int targetIndex = m_wm->getGroundSensorValue();
    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset);
    }
}

void CoopFixed2AgentObserver::reset()
{
    AgentObserver::reset();
}
