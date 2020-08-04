/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef DUMMYAGENTOBSERVER_H
#define DUMMYAGENTOBSERVER_H

#include "Observers/AgentObserver.h"

class RobotWorldModel;

class DummyAgentObserver : public AgentObserver
{
	public:
		DummyAgentObserver(RobotWorldModel *wm);
		~DummyAgentObserver();

		void reset();
		void stepPre();
};

#endif

