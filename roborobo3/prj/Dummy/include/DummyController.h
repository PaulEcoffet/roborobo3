/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#ifndef DUMMYCONTROLLER_H
#define DUMMYCONTROLLER_H

#include "Controllers/Controller.h"

class RobotWorldModel;

class DummyController : public Controller
{
    
public:
    
    DummyController(RobotWorldModel *wm);
    ~DummyController();
    
    void reset();
    void step();
};


#endif

