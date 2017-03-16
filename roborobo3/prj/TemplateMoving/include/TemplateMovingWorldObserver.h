/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef TEMPLATEMOVINGWORLDOBSERVER_H
#define TEMPLATEMOVINGWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Observers/WorldObserver.h"

class World;

class TemplateMovingWorldObserver : public WorldObserver
{
	protected:
		
	public:
		TemplateMovingWorldObserver( World *__world );
		~TemplateMovingWorldObserver();
				
		void reset();
		void step();
		
};

#endif

