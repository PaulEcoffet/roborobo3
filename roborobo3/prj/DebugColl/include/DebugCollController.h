/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef DEBUGCOLLCONTROLLER_H
#define DEBUGCOLLCONTROLLER_H

#include "Controllers/Controller.h"

class RobotWorldModel;

class DebugCollController : public Controller
{
	public:
		//Initializes the variables
		DebugCollController( RobotWorldModel *__wm );
		~DebugCollController();
		
		void reset();
		void step();
    
        void monitorSensory();
};


#endif

