/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#ifndef MOVINGEECONTROLLER_H
#define MOVINGEECONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Graphics.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "MovingEE/include/MovingEEAgentObserver.h"
#include <neuralnetworks/NeuralNetwork.h>
#include "TemplateEE/include/TemplateEEController.h"

#include <iomanip>

using namespace Neural;


class MovingEEController : public TemplateEEController
{
    public:
    
        MovingEEController(RobotWorldModel *wm);
        ~MovingEEController();
    
        double getFitness();

    protected:
    
        void initController();
        void stepController();
    
        void performSelection();
        void performVariation();
    
        void broadcastGenome();
    
        void resetFitness();
        void updateFitness();

        void logCurrentState();
};


#endif

