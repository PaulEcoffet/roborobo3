/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "TemplateMoving/include/TemplateMovingWorldObserver.h"

#include "World/World.h"


TemplateMovingWorldObserver::TemplateMovingWorldObserver( World *__world ) : WorldObserver( __world )
{
	_world = __world;
}

TemplateMovingWorldObserver::~TemplateMovingWorldObserver()
{
	// nothing to do.
}

void TemplateMovingWorldObserver::reset()
{
	// nothing to do.
}

void TemplateMovingWorldObserver::step()
{
	// nothing to do.
}
