/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef TEMPLATEMOVINGCONTROLLER_H
#define TEMPLATEMOVINGCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Controllers/Controller.h"

#include "WorldModels/RobotWorldModel.h"

class TemplateMovingController : public Controller
{
public:
    //Initializes the variables
    TemplateMovingController(RobotWorldModel *__wm);

    ~TemplateMovingController();

    void reset();

    void step();

    void monitorSensory();
};


#endif

