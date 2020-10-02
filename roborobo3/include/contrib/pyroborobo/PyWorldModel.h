//
// Created by Paul Ecoffet on 08/06/2020.
//

#ifndef ROBOROBO3_PYWORLDMODEL_H
#define ROBOROBO3_PYWORLDMODEL_H

#include "WorldModels/RobotWorldModel.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

class PyWorldModel : public RobotWorldModel
{
public:
    using RobotWorldModel::RobotWorldModel;

    py::tuple getRobotGroundSensors()
    {
        return py::make_tuple(getGroundSensor_redValue(), getGroundSensor_greenValue(), getGroundSensor_blueValue());
    }

    py::array_t<double> getCameraSensorsDist()
    {
        py::array_t<double> res(_cameraSensorsNb);
        auto fastres = res.mutable_unchecked();
        for (int i = 0; i < _cameraSensorsNb; i++)
        {
            fastres(i) = getNormalizedDistanceValueFromCameraSensor(i);
        }
        return res;
    }

    py::array_t<int> getCameraObjectIds()
    {
        py::array_t<int> res(_cameraSensorsNb);
        auto fastres = res.mutable_unchecked();
        for (int i = 0; i < _cameraSensorsNb; i++)
        {
            fastres(i) = getObjectIdFromCameraSensor(i);
        }
        return res;
    }

    py::array_t<double> getCameraAngles()
    {
        py::array_t<double> res(_cameraSensorsNb);
        auto fastres = res.mutable_unchecked();
        for (int i = 0; i < _cameraSensorsNb; i++)
        {
            fastres(i) = getCameraSensorTargetAngle(i);
        }
        return res;
    }
};

#endif //ROBOROBO3_PYWORLDMODEL_H
