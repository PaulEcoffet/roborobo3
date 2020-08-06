/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_PYNEGOTIATECONTROLLER_H
#define ROBOROBO3_PYNEGOTIATECONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "PyNegotiateOpportunity.h"
#include "PyNegotiateWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the PyNegotiate experiment. Very similar to the PyNegotiateController.
 */

class PyNegotiateController : public Controller
{
public:
    explicit PyNegotiateController(RobotWorldModel *wm);

    ~PyNegotiateController() override;

    void step() override;

    void reset() override;

    void resetFitness();

    void updateFitness(double newFitness);

    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

protected:
    PyNegotiateWorldModel *m_wm;
    static std::vector<std::string> inputnames;

    bool fill_names = false;
    int verbose = 0;

    void fillNames();

    void wander_behavior() const;

};


#endif //ROBOROBO3_PYNEGOTIATECONTROLLER_H
