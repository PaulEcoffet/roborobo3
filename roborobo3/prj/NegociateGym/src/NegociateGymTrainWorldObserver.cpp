/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <NegociateGym/include/NegociateGymWorldModel.h>
#include <NegociateGym/include/NegociateGymTrainWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "NegociateGym/include/NegociateGymTrainWorldObserver.h"
#include "NegociateGym/include/NegociateGymController.h"
#include "NegociateGym/include/NegociateGymSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
//#include <cv.hpp>

using boost::algorithm::clamp;


void NegociateGymTrainWorldObserver::computeOpportunityImpacts()
{
    const double b = NegociateGymSharedData::b;
    // Mark all robots as not on an cooperation opportunity
    mark_all_robots_as_alone();

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<NegociateGymOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (n != 0)
        {
            if (NegociateGymSharedData::fixRobotNb && n > 2)
            {
                n = 2;
            }

            mark_robots_on_opp(opp);

            // If we only give reward for the first two robots
            if (NegociateGymSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
            {
                itmax = opp->getNearbyRobotIndexes().begin() + 2;
            }

            bool everyone_agree = true;

            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *const wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                auto *const ctl = dynamic_cast<NegociateGymController *>(m_world->getRobot(*index)->getController());
                double coop = wm->getCoop();
                totalInvest += coop;
                totalA += wm->selfA;
                everyone_agree = everyone_agree && ctl->acceptPlay();
            }


            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *wm = dynamic_cast<NegociateGymWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                wm->appendOwnInvest(coop);

                if (NegociateGymSharedData::onlyOtherInTotalInv)
                {
                    wm->appendTotalInvest(totalInvest - coop);
                }
                else
                {
                    wm->appendTotalInvest(totalInvest);
                }

                if (n >= 2)
                {
                    if (opp->getXCenterPixel() + opp->getYCenterPixel() * 100 != wm->lastvisit)
                    {
                        wm->_fitnessValue++;
                    }
                    wm->lastvisit = opp->getXCenterPixel() + opp->getYCenterPixel() * 100;

                    /*
                    wm->setAlive(false);

                    m_world->getRobot(*index)->unregisterRobot();
                    m_world->getRobot(*index)->setCoord(0, 0);
                    m_world->getRobot(*index)->setCoordReal(0, 0);
                    m_world->getRobot(*index)->registerRobot();
                     */
                    opp->kill();
                }
            }

        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;
    }
}

