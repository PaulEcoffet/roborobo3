/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <Negociate/include/NegociateWorldModel.h>
#include <Negociate/include/NegociateTrainWorldObserver.h>
#include <boost/algorithm/string.hpp>
#include "contrib/json/json.hpp"
#include "Utilities/Graphics.h"
#include "RoboroboMain/main.h"
#include "Negociate/include/NegociateTrainWorldObserver.h"
#include "Negociate/include/NegociateController.h"
#include "Negociate/include/NegociateSharedData.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/math/distributions/normal.hpp>
//#include <cv.hpp>

using boost::algorithm::clamp;


void NegociateTrainWorldObserver::computeOpportunityImpacts()
{
    const double b = NegociateSharedData::b;
    // Mark all robots as not on an cooperation opportunity
    mark_all_robots_as_alone();

    for (auto *physicalObject : gPhysicalObjects)
    {
        double totalInvest = 0;
        double totalA = 0;
        auto *opp = dynamic_cast<NegociateOpportunity *>(physicalObject);
        auto itmax = opp->getNearbyRobotIndexes().end();

        int n = opp->getNbNearbyRobots();
        if (n != 0)
        {
            if (NegociateSharedData::fixRobotNb && n > 2)
            {
                n = 2;
            }

            mark_robots_on_opp(opp);

            // If we only give reward for the first two robots
            if (NegociateSharedData::fixRobotNb && opp->getNbNearbyRobots() > 2)
            {
                itmax = opp->getNearbyRobotIndexes().begin() + 2;
            }

            bool everyone_agree = true;

            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *const wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                auto *const ctl = dynamic_cast<NegociateController *>(m_world->getRobot(*index)->getController());
                double coop = wm->getCoop();
                totalInvest += coop;
                totalA += wm->selfA;
                everyone_agree = everyone_agree && ctl->acceptPlay();
            }


            for (auto index = opp->getNearbyRobotIndexes().begin(); index != itmax; index++)
            {
                auto *wm = dynamic_cast<NegociateWorldModel *>(m_world->getRobot(*index)->getWorldModel());
                double coop = wm->getCoop();
                wm->appendOwnInvest(coop);

                if (NegociateSharedData::onlyOtherInTotalInv)
                {
                    wm->appendTotalInvest(totalInvest - coop);
                }
                else
                {
                    wm->appendTotalInvest(totalInvest);
                }

                if(n >= 2)
                {
                    wm->_fitnessValue++;
                    /*wm->setAlive(false);h

                    m_world->getRobot(*index)->unregisterRobot();
                    m_world->getRobot(*index)->setCoord(0, 0);
                    m_world->getRobot(*index)->setCoordReal(0, 0);
                    m_world->getRobot(*index)->registerRobot();*/
                    opp->kill();
                }
            }

        }

        // Set the cur total invest for coloring
        opp->curInv = totalInvest;
        opp->curA = totalA / n;
    }
}

