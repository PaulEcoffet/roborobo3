#include "World/MovingObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include "../prj/MovingNS/include/MovingNSController.h"
#include "../prj/MonoRobot/include/MonoRobotController.h"
#include "../prj/SingleGenome/include/SingleGenomeController.h"
#include "../prj/TemplateMoving/include/TemplateMovingController.h"
#include "../prj/CoopOpportunity2Max/include/CoopOpportunity2MaxController.h"



#include <iomanip>

MovingObject::MovingObject( int __id ) : CircleObject ( __id )
{
    setType(5);
    for (auto& totEff: _totalEfforts)
        totEff = 0;
}

void MovingObject::reset()
{
    for (auto& totEff: _totalEfforts)
        totEff = 0;
    resetLocation();
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
            _efforts.insert(std::pair<int, double>(imp.first, vr)); // vr is here in fact the cooperation level (and vtheta as well)
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
                
                if (canRegisterDynamic(newX, newY))
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
//    printf("Iteration %d: running step() of object %d with %lu robots around\n", gWorld->getIterations(), _id, _nearbyRobots.size());
    double oldX = _xReal, oldY = _yReal;

    move(); //handles movement, and sets _didMove

//  double movement = sqrt((oldX-_desiredX)*(oldX-_desiredX) + (oldY-_desiredY)*(oldY-_desiredY));
	_nbNearbyRobots = static_cast<int>(_nearbyRobots.size());
    
    if (gStuckMovableObjects) // get back into place!
    {
        _xReal = oldX;
        _yReal = oldY;
    }
    
    // sum of the norms of the impulses given to the object
    double totalEffort = 0;
    for (auto eff: _efforts)
	{
        totalEffort += eff.second;
	}

    // remember the total effort of the current iteration
    _totalEfforts[gWorld->getIterations()%_memorySize] = totalEffort;
    
    for (auto robotID: _nearbyRobots)
    {
        Robot *robot = gWorld->getRobot(robotID);
    
        // see how much the robot helped push us
        double effort = 0;
        if (_efforts.count(robotID+gRobotIndexStartOffset) > 0)
        {
            effort = _efforts[robotID+gRobotIndexStartOffset];
        }
        std::string projectName = gProperties.getProperty("ConfigurationLoaderObjectName");
        
        if (projectName == "MovingNSConfigurationLoader")
        {
            MovingNSController *ctl = dynamic_cast<MovingNSController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, _nbNearbyRobots);
        }
        else if (projectName == "MonoRobotConfigurationLoader")
        {
            MonoRobotController *ctl = dynamic_cast<MonoRobotController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, _nbNearbyRobots);
        }
        else if (projectName == "SingleGenomeConfigurationLoader")
        {
            SingleGenomeController *ctl = dynamic_cast<SingleGenomeController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, _nbNearbyRobots);
        }
        else if (projectName == "CoopOpportunity2MaxConfigurationLoader")
        {
            CoopOpportunity2MaxController *ctl = dynamic_cast<CoopOpportunity2MaxController *>(robot->getController());
            ctl->wasNearObject(_id, _didMove, totalEffort, effort, _nbNearbyRobots);
            
        }
    }
    _nearbyRobots.clear();
    _efforts.clear();
    stepPhysicalObject();
}

bool MovingObject::canRegisterDynamic( Sint16 __x, Sint16 __y )
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

bool MovingObject::canRegisterStatic(Sint16 __x, Sint16 __y)
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
                    return false; // collision!
                }
            }
        }
    }
    
    //  test footprint (pixels from both ground image and environment image must be empty)
    for (Sint16 xColor = __x - _footprintRadius ; xColor < __x + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = __y - _footprintRadius ; yColor < __y + Sint16 (_footprintRadius); yColor ++)
        {
            if ( pow(xColor-__x,2) + pow(yColor - __y,2) < _footprintRadius*_footprintRadius )
            {
                Uint32 pixelFootprint = getPixel32_secured( gFootprintImage, xColor, yColor);
                Uint32 pixelEnvironment = getPixel32_secured( gEnvironmentImage, xColor, yColor);
                if (
                    pixelEnvironment != SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ||
                    ( gFootprintImage_restoreOriginal == true  && pixelFootprint != getPixel32_secured( gFootprintImageBackup, xColor, yColor ) ) || // case: ground as initialized or rewritten (i.e. white)
                    ( gFootprintImage_restoreOriginal == false && pixelFootprint != SDL_MapRGBA( gFootprintImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ) // case: only white ground
                    )
                    return false; // collision!
            }
        }
    }
    
    return true;
}

bool MovingObject::canRegister()
{
    return canRegisterStatic(getXCenterPixel(), getYCenterPixel());
}

void MovingObject::show() {
    //	printf("Displaying moving object #%d\n", _id);
    CircleObject::show();
}

double MovingObject::getRecentTotalEffort()
{
    double res = 0;
    for (auto totEff: _totalEfforts)
        res += totEff;
    return res;
}

// Here, the robot ID has the gRobotStartOffset added because we might be pushed by either robots or other objects
void MovingObject::isPushed( int __idAgent, std::tuple<double, double> __speed )
{
    if (_impulses.count(__idAgent) == 0) {
        _impulses.insert(std::pair<int, std::tuple<double, double>>(__idAgent, __speed));
        _nearbyRobots.insert(__idAgent-gRobotIndexStartOffset);
    }
}

void MovingObject::isTouched( int __idAgent )
{
    
}

void MovingObject::isWalked( int __idAgent )
{
//    _nearbyRobots.insert(__idAgent);
}
