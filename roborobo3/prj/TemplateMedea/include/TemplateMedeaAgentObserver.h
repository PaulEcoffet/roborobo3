/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef TEMPLATEMEDEAAGENTOBSERVER_H
#define TEMPLATEMEDEAAGENTOBSERVER_H

#include "TemplateEE/include/TemplateEEAgentObserver.h"

class RobotWorldModel;

class TemplateMedeaAgentObserver : public TemplateEEAgentObserver
{
	public:
        TemplateMedeaAgentObserver( RobotWorldModel *wm );
		~TemplateMedeaAgentObserver();

        virtual void stepPre();
};

#endif

