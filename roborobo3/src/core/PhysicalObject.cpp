/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "World/PhysicalObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

PhysicalObject::PhysicalObject( int __id ) // a unique and consistent __id should be given as argument
{
    _id = __id;
    init();
}

void PhysicalObject::init()
{
    double x = 0.0, y = 0.0;
    
	std::string s = "";
	std::stringstream out;
	out << getId();
    
    regrowTime = 0;
    
    s = "physicalObject[";
	s += out.str();
	s += "].regrowTimeMax";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<int>(regrowTimeMax, gProperties.getProperty( s ), std::dec);
	}
    else
    {
        regrowTimeMax = gPhysicalObjectDefaultRegrowTimeMax;
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].overwrite";
	if ( gProperties.hasProperty( s ) )
        gProperties.checkAndGetPropertyValue(s,&overwrite,true);
    else
    {
        overwrite = gPhysicalObjectDefaultOverwrite;
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].relocate";
	if ( gProperties.hasProperty( s ) )
        gProperties.checkAndGetPropertyValue(s,&relocate,true);
    else
    {
        relocate = gPhysicalObjectDefaultRelocate;
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].visible";
	if ( gProperties.hasProperty( s ) )
        gProperties.checkAndGetPropertyValue(s,&_visible,true);
	else
        gProperties.checkAndGetPropertyValue("gPhysicalObjectsVisible", &_visible, true);
    
    Uint32 colorValue;
    
    s = "physicalObject[";
	s += out.str();
	s += "].displayColorRed";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s ), std::dec);
        _displayColorRed = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorRed = gPhysicalObjectDefaultDisplayColorRed; // default
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].displayColorGreen";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s ), std::dec);
        _displayColorGreen = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorGreen = gPhysicalObjectDefaultDisplayColorGreen; // default
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].displayColorBlue";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s ), std::dec);
        _displayColorBlue = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorBlue = gPhysicalObjectDefaultDisplayColorBlue; // default
    }
    
    s = "physicalObject[";
	s += out.str();
	s += "].x";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<double>(x, gProperties.getProperty( s ), std::dec);
	}
	else
	{
        x = -1.0;
	}
    
	s = "physicalObject[";
	s += out.str();
	s += "].y";
	if ( gProperties.hasProperty( s ) )
	{
		convertFromString<double>(y, gProperties.getProperty( s ), std::dec);
	}
	else
	{
        y = -1.0;
	}
    
    setCoordinates( x, y );
}

int PhysicalObject::findRandomLocation()
{
    return findRandomLocation(gPhysicalObjectsInitAreaX, gPhysicalObjectsInitAreaWidth+gPhysicalObjectsInitAreaX, gPhysicalObjectsInitAreaY, gPhysicalObjectsInitAreaHeight+gPhysicalObjectsInitAreaY);
}

int PhysicalObject::findRandomLocation( int __xMin, int __xMax, int __yMin, int __yMax)
{
    double x = 0.0, y = 0.0;
    
    int tries = 0;
    
    do {
        x = rand() % (__xMax - __xMin) + __xMin;
        y = rand() % (__yMax - __yMin) + __yMin;
                
        setCoordinates( x, y );
        
        tries++;
    } while ( canRegister() == false && tries < gLocationFinderMaxNbOfTrials );
    
    if ( tries == gLocationFinderMaxNbOfTrials )
    {
        std::cerr << "[CRITICAL] Random initialization of initial position for physical object #" << getId() << " after trying " << gLocationFinderMaxNbOfTrials << " random picks (all failed). There may be too few (none?) possible locations (you may try to manually set initial positions). EXITING.\n";
        exit(-1);
    }
    
    if ( gVerbose )
    {
        std::cout << "[INFO] Object " << _id << " positioned at (" << _xReal << "), ";
        std::cout<< _yReal << " -- [random pick after " << tries << " tries]" << std::endl;
    }
        
    return tries;
}

void PhysicalObject::stepPhysicalObject()
{
    if ( registered == false && regrowTime != -1 )
    {
        if ( regrowTime > 0 )
        {
            regrowTime--;
        }
        else
        {
            if ( relocate == true )
            {
                findRandomLocation(); // exit on fail.
                _visible = true;
                registered = true;
                registerObject();
            }
            else
            {
                if ( canRegister() == true || overwrite == true )
                {
                    _visible = true;
                    registered = true;
                    registerObject();
                }
            }
        }
    }
    else
    {
        if ( _visible )
        {
            if ( gPhysicalObjectsRedraw == true )
            {
                registerObject();
            }
        }
    }
}

bool PhysicalObject::isInstanceOf ( int index ) // static
{
    if ( index >= gPhysicalObjectIndexStartOffset && index <  gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )
        return true;
    else
        return false;
}


/*
 Given an object not registered in the environment, tries to re-register at once.
 Return false (and do nothing) if not possible at desired location.
 Exit if function is called when it should not -- calling this function can be critical, including registering multiple instances of the same object, thus breaking consistency.
*/
bool PhysicalObject::triggerRegrow()
{
    if ( registered == true )  // exit if object is already registered in the environment
    {
        std::cerr << "[CRITICAL] physical object #" << getId() << ": attempt to call triggerRegrow() method while object already exists in the environment. EXITING.\n";
        exit(-1);
    }
    
    if ( canRegister() == false ) // check is position is free.
    {
        return false;
    }
    else
    {
        regrowTime = gPhysicalObjectDefaultRegrowTimeMax;
        _visible = true;
        registered = true;
        registerObject();
    }
    
    return true;
}

void PhysicalObject::resetLocation()
{
    std::string loc = "physicalObject[" + std::to_string(_id) + "]";
    if (gProperties.hasProperty(loc + ".x") && gProperties.hasProperty(loc + ".y"))
    {
        convertFromString<double>(_xReal, gProperties.getProperty(loc+".x"), std::dec);
        convertFromString<double>(_yReal, gProperties.getProperty(loc+".y"), std::dec);
    }
    else
        findRandomLocation();
}

std::string PhysicalObject::inspect()
{
    return std::string("PhysicalObject::inspect() not implemented.\n");
}