/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef TEMPLATERANDOMWALKWORLDOBSERVER_H
#define TEMPLATERANDOMWALKWORLDOBSERVER_H

#include "Observers/WorldObserver.h"

class World;

class TemplateRandomwalkWorldObserver : public WorldObserver
{
	protected:
		
	public:
		TemplateRandomwalkWorldObserver( std::shared_ptr<World> __world );
		~TemplateRandomwalkWorldObserver();
				
		void initPre();
        void initPost();

        void stepPre();
        void stepPost();
		
};

#endif

