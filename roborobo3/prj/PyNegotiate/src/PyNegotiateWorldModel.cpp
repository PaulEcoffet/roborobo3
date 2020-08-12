/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#include <RoboroboMain/main.h>
#include "PyNegotiate/include/PyNegotiateWorldModel.h"
#include <boost/algorithm/clamp.hpp>
#include <PyNegotiate/include/PyNegotiateSharedData.h>
#include <core/World/World.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>


using boost::algorithm::clamp;


PyNegotiateWorldModel::PyNegotiateWorldModel()
        : RobotWorldModel(),
          onOpportunity(false),
          selfA(0.5),
          opp(nullptr),
          fake(false),
          fakeCoef(1),
          teleport(false),
          punishment(0),
          spite(0)
{
    _cooperationLevel = 0;
    setNewSelfA();
}

void PyNegotiateWorldModel::setNewSelfA()
{

    // set the selfA, it makes a non-gaussian ESS distribution though TODO see if skewed is okay
    selfA = std::max(randgaussian() * PyNegotiateSharedData::stdA + PyNegotiateSharedData::meanA, 0.01);
}


void PyNegotiateWorldModel::reset()
{
    onOpportunity = false;
    teleport = false;
    toBeTeleported = false;
    opp = nullptr;
    reward = 0;
}


bool PyNegotiateWorldModel::isPlaying() const
{
    return onOpportunity && (arrival <= 2 || !PyNegotiateSharedData::fixRobotNb) &&
           (!PyNegotiateSharedData::atLeastTwo || nbOnOpp >= 2);
}

double PyNegotiateWorldModel::getCoop(bool trueValue) const
{
    double coop = _cooperationLevel;
    if (!trueValue)
    {
        coop = clamp(coop + fakeCoef, 0, PyNegotiateSharedData::maxCoop);
    }
    return coop;
}

py::object PyNegotiateWorldModel::getObservations()
{
    py::dict obs;
    std::vector<double> inputs(_cameraSensorsNb *
    4 + 2, 0);
    const int WALL_ID = 0;
    int i = 0;

    /*
     * Camera inputs
     */
    for (int j = 0; j < _cameraSensorsNb; j++)
    {
        bool isOpportunity = false;
        double cur_nbOnOpp = 0;
        auto entityId = static_cast<int>(getObjectIdFromCameraSensor(j));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<PyNegotiateOpportunity *>(
                    gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (opportunity == opp)
            {
                cur_nbOnOpp = nbOnOpp;
            }
            else
            {
                cur_nbOnOpp = opportunity->getNbNearbyRobots();
            }
        }
        double dist = getDistanceValueFromCameraSensor(j) / getCameraSensorMaximumDistanceValue(j);
        inputs[i++] = (Agent::isInstanceOf(entityId)) ? dist : 1;
        inputs[i++] = (entityId == WALL_ID) ? dist : 1;
        inputs[i++] = (isOpportunity) ? dist : 1;
        inputs[i++] = cur_nbOnOpp / 10.0;
    }

    /*
     * Game Inputs
     */

    if (opp != nullptr)
    {
        inputs[i++] = (opp->getAllCoop() - getCoop()) / PyNegotiateSharedData::maxCoop;
    }
    else
    {
        inputs[i++] = 0;
    }
    inputs[i++] = getCoop() / PyNegotiateSharedData::maxCoop;

    obs["seeking"] = seeking || wasSeekerLastStep;
    obs["obs"] = py::array(py::cast(inputs));
    return std::move(obs);
}

static inline double normalize(const double value, const double min = 0, const double max = 1)
{
    return ((value - min) / (max - min));
}

void PyNegotiateWorldModel::setActions(const py::object &actions)
{
    auto actions_vec = actions.cast<std::vector<double>>();
    assert(std::all_of(actions_vec.begin(), actions_vec.end(), [](double a) { return !isnan(a); }));
    _desiredTranslationalValue = actions_vec[0] * gMaxTranslationalSpeed;
    _desiredRotationalVelocity = actions_vec[1] * gMaxRotationalSpeed;
    accept = actions_vec[2] > 0;
    const double normalized_coop_action = normalize(actions_vec[3], -1, 1);
    _cooperationLevel = normalized_coop_action * PyNegotiateSharedData::maxCoop;
}

bool PyNegotiateWorldModel::getDone()
{
    return !seeking;
}

double PyNegotiateWorldModel::getReward()
{
    return reward;
}
