/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef DEBUGCOLLWORLDOBSERVER_H
#define DEBUGCOLLWORLDOBSERVER_H

#include "Observers/WorldObserver.h"

class World;

class DebugCollWorldObserver : public WorldObserver
{
	protected:
		
	public:
		DebugCollWorldObserver( World *__world );
		~DebugCollWorldObserver();
				
		void reset();

		std::ofstream log;


	void stepPre();
		void stepPost();
    
};

#endif

