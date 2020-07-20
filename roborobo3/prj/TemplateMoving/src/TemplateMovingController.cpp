/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "TemplateMoving/include/TemplateMovingController.h"

TemplateMovingController::TemplateMovingController(RobotWorldModel *__wm) : Controller(__wm)
{
    // nothing to do
}

TemplateMovingController::~TemplateMovingController()
{
    // nothing to do.
}

void TemplateMovingController::reset()
{
    // nothing to do.
}

void TemplateMovingController::step()
{
    // a basic obstacle avoidance behavior

    _wm->_desiredTranslationalValue = +1 - ((double) gSensorRange -
                                            ((_wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) +
                                              _wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE)) / 2)) /
                                           (double) gSensorRange;
    if (_wm->getCameraSensorValue(0, SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(1, SENSOR_DISTANCEVALUE) +
        _wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) <
        _wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
        _wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE))
        _wm->_desiredRotationalVelocity = +5;
    else if (_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
             _wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE) < 3 * gSensorRange)
        _wm->_desiredRotationalVelocity = -5;
    else if (_wm->_desiredRotationalVelocity > 0)
        _wm->_desiredRotationalVelocity--;
    else if (_wm->_desiredRotationalVelocity < 0)
        _wm->_desiredRotationalVelocity++;
    else
        _wm->_desiredRotationalVelocity = 0.01 - (double) (randint() % 10) / 10. * 0.02;

}
