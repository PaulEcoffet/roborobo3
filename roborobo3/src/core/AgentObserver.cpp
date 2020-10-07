/*
 *  AgentObserver.cpp
 *  roborobo-online
 *
 *  Created by Nicolas on 24/03/09.
 *  Copyright 2009. All rights reserved.
 *
 */

#include "Observers/AgentObserver.h"

AgentObserver::AgentObserver( )
{
}

AgentObserver::AgentObserver(std::shared_ptr<RobotWorldModel> __wm)
{
    _wm = __wm;
}

AgentObserver::~AgentObserver()
{
	// nothing to do.
}

void AgentObserver::reset()
{
	// nothing to do.
}

void AgentObserver::stepPre()
{
    // nothing to do.
}

void AgentObserver::stepPost()
{
    // nothing to do.
}

std::shared_ptr<Controller> AgentObserver::getController()
{
    return gWorld->getRobot(_wm->getId())->getController();
}
