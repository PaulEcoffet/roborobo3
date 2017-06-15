#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include "../prj/MovingNS/include/MovingNSController.h"
#include "../prj/MonoRobot/include/MonoRobotController.h"
#include "../prj/SingleGenome/include/SingleGenomeController.h"
#include "../prj/TemplateMoving/include/TemplateMovingController.h"



#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
    setType(5);
}

void MovingObject::move() {
    _hitWall = false;
    _didMove = false;
    _desiredX = _xReal;
    _desiredY = _yReal;
    if (_impulses.size() > 0)
    {
        //       printf("[DEBUG] Moving object %d\n", _id);
        double impXtot = 0, impYtot = 0, vr, vtheta, vx, vy, ux, uy;
        
        for (auto& imp : _impulses) {
            // We only want the component of the speed normal to the centers of mass
            // v: agent speed vector, u: agent-object vector
            // impulses are in polar form
            std::tie(vr, vtheta) = imp.second;
            vx = vr*cos(vtheta * M_PI / 180.0);
            vy = vr*sin(vtheta * M_PI / 180.0);
            
            if (imp.first >= gRobotIndexStartOffset) { // robot
                Robot *robot = gWorld->getRobot(imp.first-gRobotIndexStartOffset);
                ux = _xReal - robot->getWorldModel()->getXReal();
                uy = _yReal - robot->getWorldModel()->getYReal();
                
            }
            else // other object
            {
                PhysicalObject *object = gPhysicalObjects[imp.first];
                ux = _xReal - object->getXReal();
                uy = _yReal - object->getYReal();
            }
            double sqnorm = ux*ux + uy*uy;
            double impX =(vx*ux+vy*uy)*ux/sqnorm;
            double impY = (vx*ux+vy*uy)*uy/sqnorm;
            impXtot += impX;
            impYtot += impY;
            _efforts.insert(std::pair<int, double>(imp.first, sqrt(impX*impX+impY*impY)));
        }
        
        _desiredLinearSpeed = sqrt(impXtot*impXtot + impYtot*impYtot);
        _desiredSpeedOrientation = atan2(impYtot, impXtot) * 180 / M_PI;
        _desiredX = _xReal+impXtot;
        _desiredY = _yReal+impYtot;
        
        Sint16 newX = _desiredX; //rounded
        Sint16 newY = _desiredY;
        
        if (gStuckMovableObjects == false) {
            if (newX != getXCenterPixel() || newY != getYCenterPixel()) // we're going to try to move onscreen
            {
                unregisterObject();
                hide();
                
                if (canRegister(newX, newY))
                {
                    _xReal = _desiredX;
                    _yReal = _desiredY;
                    _didMove = true;
                }
                
                if (_hitWall) { // reappear somewhere else
                    registered = false;
                    _visible = false;
                }
                else {
                    registerObject();
                }
            }
            else // silently move offscreen by less than a pixel
            {
                _xReal = _desiredX;
                _yReal = _desiredY;
            }
        }
        _impulses.clear();
    }
}

void MovingObject::step()
{
    double oldX = _xReal, oldY = _yReal;

    move(); //handles movement, and sets _didMove

//  double movement = sqrt((oldX-_desiredX)*(oldX-_desiredX) + (oldY-_desiredY)*(oldY-_desiredY));
	int nbRobots = static_cast<int>(_nearbyRobots.size());
    
    if (gStuckMovableObjects) // get back into place!
    {
        _xReal = oldX;
        _yReal = oldY;
    }
    
    // sum of the norms of the impulses given to the object
    /* In this experiment, we say that each robot that helped gave us a 1 */
    double totalEffort = 0;
    for (auto eff: _efforts)
        totalEffort += 0.5;

    for (auto robotID: _nearbyRobots)
    {
        Robot *robot = gWorld->getRobot(robotID);
    
        // see how much the robot helped push us
        double effort = 0;
        if (_efforts.count(robotID+gRobotIndexStartOffset) > 0)
        {
            effort = 0.5;
        }
        std::string projectName = gProperties.getProperty("ConfigurationLoaderObjectName");
        
        if (projectName == "MovingNSConfigurationLoader")
        {
            MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, nbRobots);
        }
        else if (projectName == "MonoRobotConfigurationLoader")
        {
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, nbRobots);
        }
        else if (projectName == "SingleGenomeConfigurationLoader")
        {
            SingleGenomeController *ctl = dynamic_cast<SingleGenomeController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, nbRobots);
        }
    }
    _nearbyRobots.clear();
    _efforts.clear();
    stepPhysicalObject();
}

bool MovingObject::canRegister( Sint16 __x, Sint16 __y )
{
    // test shape
    for (Sint16 xColor = __x - _radius ; xColor < __x + _radius ; xColor++)
    {
        for (Sint16 yColor = __y - _radius ; yColor < __y + _radius; yColor ++)
        {
            if ( pow (xColor-__x,2) + pow (yColor - __y,2) < _radius*_radius )
            {
                Uint32 pixel = getPixel32_secured( gEnvironmentImage, xColor, yColor);
                if ( pixel != SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ) {
                    // if we touched an object, tell it
                    Uint8 r, g, b;
                    SDL_GetRGB(pixel,gEnvironmentImage->format,&r,&g,&b);
                    
                    int targetIndex = (r<<16)+(g<<8)+b;
                    
                    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gRobotIndexStartOffset && gMovableObjects)   // physical object
                    {
                        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
                        gPhysicalObjects[targetIndex]->isPushed(_id, std::tie(_desiredLinearSpeed, _desiredSpeedOrientation));
                    } else if (targetIndex < gRobotIndexStartOffset) {
                        _hitWall = true;
                    }
                    return false; // collision!
                }
            }
        }
    }
    
    return true;
}

bool MovingObject::canRegister()
{
    return canRegister(getXCenterPixel(), getYCenterPixel());
}

void MovingObject::show() {
    //	printf("Displaying moving object #%d\n", _id);
    CircleObject::show();
}

void MovingObject::isPushed( int __idAgent, std::tuple<double, double> __speed )
{
    if (_impulses.count(__idAgent) == 0) {
        _impulses.insert(std::pair<int, std::tuple<double, double>>(__idAgent, __speed));
    }
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
    _nearbyRobots.insert(__idAgent);
}
