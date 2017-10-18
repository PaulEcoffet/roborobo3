/*
 *  Controller.cpp
 *  roborobo-online
 *
 *  Created by Nicolas on 19/03/09.
 *
 */

#include "Controllers/Controller.h"

Controller::Controller(  )
{
	// nothing to do.
}


Controller::Controller( RobotWorldModel *__wm )
{
	_wm = __wm;
}

Controller::~Controller()
{
	// nothing to do.
}

std::string Controller::inspect()
{
    return std::string("The inspect method has not been override by your controller.\n"
                       "Override the inspect method in your controller to display useful information.\n");
}
