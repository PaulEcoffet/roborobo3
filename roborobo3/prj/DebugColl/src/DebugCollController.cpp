/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "DebugColl/include/DebugCollController.h"
#include "WorldModels/RobotWorldModel.h"


DebugCollController::DebugCollController(RobotWorldModel *__wm) : Controller(__wm)
{
    // nothing to do
}

DebugCollController::~DebugCollController()
{
    // nothing to do.
}

void DebugCollController::reset()
{
    // nothing to do.
}

void DebugCollController::step()
{
    // a basic obstacle avoidance behavior

    _wm->_desiredTranslationalValue = 0.0001;
    _wm->_desiredRotationalVelocity = 60;

}
