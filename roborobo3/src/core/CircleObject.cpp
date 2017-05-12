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
		convertFromString<int>(_radius, gProperties.getProperty( s ), std::dec);
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
		convertFromString<int>(_footprintRadius, gProperties.getProperty( s ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (of super type CircleObject) missing footprint radius (integer, >=0). Check default value.\n";
        gProperties.checkAndGetPropertyValue("gPhysicalObjectDefaultFootprintRadius", &_footprintRadius, true);
    }

    double x = 0.0, y = 0.0;
    x = _xReal;
	y = _yReal;
    
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
        std::cout << "[INFO] Physical Object #" << getId() << " (of super type CircleObject) positioned at ( "<< std::setw(5) << _xReal << " , " << std::setw(5) << _yReal << " ) -- ";
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
    Sint16 _xCenterPixel = getXCenterPixel();
    Sint16 _yCenterPixel = getYCenterPixel();

    //  draw footprint
    
    Uint8 r = 0xF0;
    Uint8 g = 0xF0;
    Uint8 b = 0xF0;
    Uint32 color = SDL_MapRGBA(gScreen->format,r,g,b,SDL_ALPHA_OPAQUE);
    
    for (Sint16 xColor = _xCenterPixel - _footprintRadius ; xColor < _xCenterPixel + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _footprintRadius ; yColor < _yCenterPixel + _footprintRadius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _footprintRadius*_footprintRadius)
            {
                putPixel32_secured(gScreen, xColor, yColor,  color);
            }
        }
    }
    
    // draw object
    
    color = SDL_MapRGBA(gScreen->format,_displayColorRed,_displayColorGreen,_displayColorBlue,SDL_ALPHA_OPAQUE);
    
	for (Sint16 xColor = _xCenterPixel - _radius ; xColor < _xCenterPixel + _radius ; xColor++)
	{
		for (Sint16 yColor = _yCenterPixel - _radius ; yColor < _yCenterPixel + _radius; yColor ++)
		{
			if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _radius*_radius)
			{
                putPixel32(gScreen, xColor, yColor,  color);
			}
		}
	}
}

void CircleObject::hide()
{
    Sint16 _xCenterPixel = getXCenterPixel();
    Sint16 _yCenterPixel = getYCenterPixel();
    
    //  hide footprint (draw white)
    
    Uint8 r = 0xFF;
    Uint8 g = 0xFF;
    Uint8 b = 0xFF;
    
    Uint32 color = SDL_MapRGBA(gScreen->format,r,g,b,SDL_ALPHA_OPAQUE);
    
    for (Sint16 xColor = _xCenterPixel - _footprintRadius ; xColor < _xCenterPixel + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _footprintRadius ; yColor < _yCenterPixel + _footprintRadius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _footprintRadius*_footprintRadius)
            {
                putPixel32_secured(gScreen, xColor, yColor,  color);
            }
        }
    }
    
    // hide object (draw white)
    
	for (Sint16 xColor = _xCenterPixel - _radius ; xColor < _xCenterPixel + _radius ; xColor++)
	{
		for (Sint16 yColor = _yCenterPixel - _radius ; yColor < _yCenterPixel + _radius; yColor ++)
		{
			if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _radius*_radius)
			{
                putPixel32(gScreen, xColor, yColor,  color);
			}
		}
	}
}

bool CircleObject::canRegister()
{
    Sint16 _xCenterPixel = getXCenterPixel();
    Sint16 _yCenterPixel = getYCenterPixel();
    // test shape
    for (Sint16 xColor = _xCenterPixel - _radius ; xColor < _xCenterPixel + _radius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _radius ; yColor < _yCenterPixel + _radius; yColor ++)
        {
            if ( pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _radius*_radius )
            {
                Uint32 pixel = getPixel32_secured( gEnvironmentImage, xColor, yColor);
                if ( pixel != SDL_MapRGBA( gEnvironmentImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE ) ) {
                    return false; // collision!
                }
            }
        }
    }
    
    //  test footprint (pixels from both ground image and environment image must be empty)
    for (Sint16 xColor = _xCenterPixel - _footprintRadius ; xColor < _xCenterPixel + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _footprintRadius ; yColor < _yCenterPixel + Sint16 (_footprintRadius); yColor ++)
        {
            if ( pow(xColor-_xCenterPixel,2) + pow(yColor - _yCenterPixel,2) < _footprintRadius*_footprintRadius )
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

void CircleObject::registerObject()
{
    int id_converted = _id + gPhysicalObjectIndexStartOffset;
    Sint16 _xCenterPixel = getXCenterPixel();
    Sint16 _yCenterPixel = getYCenterPixel();
    
    //  draw footprint
    
    Uint32 color = SDL_MapRGBA( gFootprintImage->format, (Uint8)((id_converted & 0xFF0000)>>16), (Uint8)((id_converted & 0xFF00)>>8), (Uint8)(id_converted & 0xFF), SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - _footprintRadius ; xColor < _xCenterPixel + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _footprintRadius ; yColor < _yCenterPixel + _footprintRadius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _footprintRadius*_footprintRadius)
            {
                putPixel32_secured(gFootprintImage, xColor, yColor,  color);
            }
        }
    }
    
    // draw object
    
    color = SDL_MapRGBA( gEnvironmentImage->format, (Uint8)((id_converted & 0xFF0000)>>16), (Uint8)((id_converted & 0xFF00)>>8), (Uint8)(id_converted & 0xFF), SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - _radius ; xColor < _xCenterPixel + _radius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _radius ; yColor < _yCenterPixel + _radius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _radius*_radius)
            {
                putPixel32_secured(gEnvironmentImage, xColor, yColor,  color);//color);
            }
        }
    }
}

void CircleObject::unregisterObject()
{
    Sint16 _xCenterPixel = getXCenterPixel();
    Sint16 _yCenterPixel = getYCenterPixel();
    
    //  clear footprint
    
    Uint32 color = SDL_MapRGBA( gFootprintImage->format, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE );
    
    for (Sint16 xColor = _xCenterPixel - _footprintRadius ; xColor < _xCenterPixel + _footprintRadius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _footprintRadius ; yColor < _yCenterPixel + _footprintRadius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _footprintRadius*_footprintRadius)
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
    
    for (Sint16 xColor = _xCenterPixel - _radius ; xColor < _xCenterPixel + _radius ; xColor++)
    {
        for (Sint16 yColor = _yCenterPixel - _radius ; yColor < _yCenterPixel + _radius; yColor ++)
        {
            if (pow (xColor-_xCenterPixel,2) + pow (yColor - _yCenterPixel,2) < _radius*_radius)
            {
                putPixel32_secured(gEnvironmentImage, xColor, yColor,  color);//color);
            }
        }
    }
}
