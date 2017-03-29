#include "World/CircleObject.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include <iomanip>
#include <algorithm>

CircleObject::CircleObject( int __id ) : PhysicalObject( __id ) // a unique and consistent __id should be given as argument
{
	std::string s = "";
	std::stringstream out;
	out << getId();
    
    s = "physicalObject[";
	s += out.str();
	s += "].radius";
	if ( gProperties.hasProperty( s ) )
		convertFromString<double>(_radius, gProperties.getProperty( s ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (of super type CircleObject) missing radius (integer, >=0). Check default value.\n";
        gProperties.checkAndGetPropertyValue("gPhysicalObjectDefaultRadius", &_radius, true);
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].footprintRadius";
	if ( gProperties.hasProperty( s ) )
		convertFromString<double>(_footprintRadius, gProperties.getProperty( s ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (of super type CircleObject) missing footprint radius (integer, >=0). Check default value.\n";
        gProperties.checkAndGetPropertyValue("gPhysicalObjectDefaultFootprintRadius", &_footprintRadius, true);
    }

    double x = 0.0, y = 0.0;
    x = _xCenterPixel;
	y = _yCenterPixel;
    
    int tries = 0;
    bool randomLocation = false;

    if ( x == -1.0 || y == -1.0 )
    {
        tries = tries + findRandomLocation();
        randomLocation = true;
    }
    else
    {
        if ( canRegister() == false )  // i.e. user-defined location, but cannot register. Pick random.
        {
            std::cerr << "[CRITICAL] cannot use user-defined initial location (" << x << "," << y << ") for physical object #" << getId() << " (localization failed). EXITING.";
            exit(-1);
        }
        randomLocation = false;
    }
    
    if ( gVerbose )
    {
        std::cout << "[INFO] Physical Object #" << getId() << " (of super type CircleObject) positioned at ( "<< std::setw(5) << _xCenterPixel << " , " << std::setw(5) << _yCenterPixel << " ) -- ";
        if ( randomLocation == false )
            std::cout << "[user-defined position]\n";
        else
        {
            std::cout << "[random pick after " << tries;
            if ( tries <= 1 )
                std::cout << " try]";
            else
                std::cout << " tries]";
            std::cout << "\n";
        }
    }
    
    if ( _visible )
    {
        registerObject();
    }
    
    registered = true;
}


void CircleObject::show() // display on screen (called in the step() method if gDisplayMode=0 and _visible=true)
{
    //  draw footprint
    
    Uint8 r = 0xF0;
    Uint8 g = 0xF0;
    Uint8 b = 0xF0;
    Uint32 color = SDL_MapRGBA(gScreen->format,r,g,b,SDL_ALPHA_OPAQUE);
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_footprintRadius) ; xColor < _xCenterPixel + Sint16(_footprintRadius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_footprintRadius) ; yColor < _yCenterPixel + Sint16 (_footprintRadius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _footprintRadius)
            {
                putPixel32_secured(gScreen, xColor, yColor,  color);
            }
        }
    }
    
    // draw object
    
    color = SDL_MapRGBA(gScreen->format,_displayColorRed,_displayColorGreen,_displayColorBlue,SDL_ALPHA_OPAQUE);
    
	for (Sint16 xColor = _xCenterPixel - Sint16(_radius) ; xColor < _xCenterPixel + Sint16(_radius) ; xColor++)
	{
		for (Sint16 yColor = _yCenterPixel - Sint16(_radius) ; yColor < _yCenterPixel + Sint16 (_radius); yColor ++)
		{
			if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _radius)
			{
                putPixel32(gScreen, xColor, yColor,  color);
			}
		}
	}
}

void CircleObject::hide()
{
    //  hide footprint (draw white)
    
    Uint8 r = 0xFF;
    Uint8 g = 0xFF;
    Uint8 b = 0xFF;
    
    Uint32 color = SDL_MapRGBA(gScreen->format,r,g,b,SDL_ALPHA_OPAQUE);
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_footprintRadius) ; xColor < _xCenterPixel + Sint16(_footprintRadius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_footprintRadius) ; yColor < _yCenterPixel + Sint16 (_footprintRadius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _footprintRadius)
            {
                putPixel32_secured(gScreen, xColor, yColor,  color);
            }
        }
    }
    
    // hide object (draw white)
    
	for (Sint16 xColor = _xCenterPixel - Sint16(_radius) ; xColor < _xCenterPixel + Sint16(_radius) ; xColor++)
	{
		for (Sint16 yColor = _yCenterPixel - Sint16(_radius) ; yColor < _yCenterPixel + Sint16 (_radius); yColor ++)
		{
			if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _radius)
			{
                putPixel32(gScreen, xColor, yColor,  color);
			}
		}
	}
}

bool CircleObject::canRegister( int __x, int __y )
{
    // test shape
    for (Sint16 xColor = __x - Sint16(_radius) ; xColor < __x + Sint16(_radius) ; xColor++)
    {
        for (Sint16 yColor = __y - Sint16(_radius) ; yColor < __y + Sint16 (_radius); yColor ++)
        {
            if ( pow (xColor-__x,2) + pow (yColor - __y,2) < _radius*_radius )
            {
                Uint32 pixel = getPixel32_secured( gEnvironmentImage, xColor, yColor);
                if ( pixel != SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ) {
                    // if we touched an object, tell it
                    Uint8 r, g, b;
                    SDL_GetRGB(pixel,gEnvironmentImage->format,&r,&g,&b);
                    
                    int targetIndex = (r<<16)+(g<<8)+b;
                    
                    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gRobotIndexStartOffset )   // physical object
                    {
                        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
                        gPhysicalObjects[targetIndex]->isPushed(_id, Point2d(_xDesiredSpeed, _yDesiredSpeed));
                    } else if (targetIndex < gRobotIndexStartOffset) {
                        _hitWall = true;
                    }
                    return false; // collision!
                }
            }
        }
    }
    
//    //  test footprint (pixels from both ground image and environment image must be empty)
//    for (Sint16 xColor = __x - Sint16(_footprintRadius) ; xColor < __x + Sint16(_footprintRadius) ; xColor++)
//    {
//        for (Sint16 yColor = __y - Sint16(_footprintRadius) ; yColor < __y + Sint16 (_footprintRadius); yColor ++)
//        {
//            if ( pow(xColor-__x,2) + pow(yColor - __y,2) < _footprintRadius*_footprintRadius )
//            {
//                Uint32 pixelFootprint = getPixel32_secured( gFootprintImage, xColor, yColor);
//                Uint32 pixelEnvironment = getPixel32_secured( gEnvironmentImage, xColor, yColor);
//                if (
//                        pixelEnvironment != SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ||
//                        ( gFootprintImage_restoreOriginal == true  && pixelFootprint != getPixel32_secured( gFootprintImageBackup, xColor, yColor ) ) || // case: ground as initialized or rewritten (i.e. white)
//                        ( gFootprintImage_restoreOriginal == false && pixelFootprint != SDL_MapRGBA( gFootprintImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ) // case: only white ground
//                   )
//                    return false; // collision!
//            }
//        }
//    }
    
    return true;
}

bool CircleObject::canRegister()
{
    return canRegister(_xCenterPixel, _yCenterPixel);
}

void CircleObject::registerObject()
{
    int id_converted = _id + gPhysicalObjectIndexStartOffset;
    
    //  draw footprint
    
    Uint32 color = SDL_MapRGBA( gFootprintImage->format, (Uint8)((id_converted & 0xFF0000)>>16), (Uint8)((id_converted & 0xFF00)>>8), (Uint8)(id_converted & 0xFF), SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_footprintRadius) ; xColor < _xCenterPixel + Sint16(_footprintRadius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_footprintRadius) ; yColor < _yCenterPixel + Sint16 (_footprintRadius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _footprintRadius)
            {
                putPixel32_secured(gFootprintImage, xColor, yColor,  color);
            }
        }
    }
    
    // draw object
    
    color = SDL_MapRGBA( gEnvironmentImage->format, (Uint8)((id_converted & 0xFF0000)>>16), (Uint8)((id_converted & 0xFF00)>>8), (Uint8)(id_converted & 0xFF), SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_radius) ; xColor < _xCenterPixel + Sint16(_radius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_radius) ; yColor < _yCenterPixel + Sint16 (_radius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _radius)
            {
                putPixel32_secured(gEnvironmentImage, xColor, yColor,  color);//color);
            }
        }
    }
}

void CircleObject::unregisterObject()
{
    //  clear footprint
    
    Uint32 color = SDL_MapRGBA( gFootprintImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_footprintRadius) ; xColor < _xCenterPixel + Sint16(_footprintRadius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_footprintRadius) ; yColor < _yCenterPixel + Sint16 (_footprintRadius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _footprintRadius)
            {
                if ( gFootprintImage_restoreOriginal == true )
                {
                    color = getPixel32_secured( gFootprintImageBackup, xColor, yColor);
                    putPixel32_secured(gFootprintImage, xColor, yColor,  color);
                }
                else
                    putPixel32_secured(gFootprintImage, xColor, yColor,  color);
            }
        }
    }
    
    // clear object
    
    color = SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - Sint16(_radius) ; xColor < _xCenterPixel + Sint16(_radius) ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - Sint16(_radius) ; yColor < _yCenterPixel + Sint16 (_radius); yColor ++)
        {
            if ((sqrt ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2))) < _radius)
            {
                putPixel32_secured(gEnvironmentImage, xColor, yColor,  color);//color);
            }
        }
    }
}

void CircleObject::step()
{
    _hitWall = false;
    if (_impulses.size() > 0)
    {
        //       printf("[DEBUG] Moving object %d\n", _id);
        double impXtot = 0, impYtot = 0, vx, vy, ux, uy;
        
        for (auto& imp : _impulses) {
            // We only want the component of the speed normal to the centers of mass
            // v: agent speed vector, u: agent-object vector
            if (imp.first >= gRobotIndexStartOffset) { // robot
                // impulses are in polar form, and angles in degrees
                Robot *robot = gWorld->getRobot(imp.first-gRobotIndexStartOffset);
                vx = imp.second.x*cos(imp.second.y * M_PI / 180.0);
                vy = imp.second.x*sin(imp.second.y * M_PI / 180.0);
                ux = (double)_xCenterPixel - robot->getWorldModel()->getXReal();
                uy = (double)_yCenterPixel - robot->getWorldModel()->getYReal();
                
            }
            else // other object
            {
                // impulse is cartesian and not polar here
                PhysicalObject *object = gPhysicalObjects[imp.first];
                vx = imp.second.x;
                vy = imp.second.y;
                ux = (double)_xCenterPixel - object->getPosition().x;
                uy = (double)_yCenterPixel - object->getPosition().y;
            }
            double sqnorm = ux*ux + uy*uy;
            impXtot += (vx*ux+vy*uy)*ux/sqnorm;
            impYtot += (vx*ux+vy*uy)*uy/sqnorm;
        }
        
        _xDesiredSpeed = impXtot;
        _yDesiredSpeed = impYtot;
                
        int dx = roundAwayFromZero(impXtot);
        int dy = roundAwayFromZero(impYtot);
                
        int newX = _xCenterPixel + dx;
        int newY = _yCenterPixel + dy;
        
        if (dx != 0 || dy != 0) // we're going to try to move
        {
            unregisterObject();
            hide();
            
            bool actuallyMoved = false;
            
            if (canRegister(newX, newY))
            {
                _xCenterPixel = newX;
                _yCenterPixel = newY;
                actuallyMoved = true;
            }
            
            if (_hitWall) { // reappear somewhere else
                registered = false;
                _visible = false;
            }
            else {
                registerObject();
                // tell robots that the push was successful (remember we did move)
                if (actuallyMoved)
                    for (auto& imp: _impulses)
                        if (imp.first >= gRobotIndexStartOffset) {
                            Robot *robot = gWorld->getRobot(imp.first-gRobotIndexStartOffset);
                            robot->getWorldModel()->setPushed(true);
                        }
            }
        }
        _impulses.clear();
    }
    stepPhysicalObject();
}

void CircleObject::push(int __idAgent, Point2d __speed)
{
    if (gMovableCircleObjects)
    {
        if (_impulses.count(__idAgent) == 0) {
            //        printf("[DEBUG] object %d is being pushed by agent %d.\n", _id, __idAgent);
            _impulses.insert(std::pair<int, Point2d>(__idAgent, __speed));
        }
    }
}
