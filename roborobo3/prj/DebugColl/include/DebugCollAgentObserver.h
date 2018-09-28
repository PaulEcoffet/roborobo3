/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */
 
#ifndef DEBUGCOLLAGENTOBSERVER_H
#define DEBUGCOLLAGENTOBSERVER_H 

#include "Observers/AgentObserver.h"

class RobotWorldModel;

class DebugCollAgentObserver : public AgentObserver
{
	public:
		DebugCollAgentObserver( );
		DebugCollAgentObserver( RobotWorldModel *__wm );
		~DebugCollAgentObserver();
				
		void reset();
		void stepPre();
		
};


#endif

