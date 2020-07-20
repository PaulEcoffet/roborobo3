/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_MAXONECONTROLLER_H
#define ROBOROBO3_MAXONECONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "MaxOneWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the MaxOne experiment. Very similar to the MaxOneController.
 */

class MaxOneController : public Controller
{
public:
    explicit MaxOneController(RobotWorldModel *wm);

    ~MaxOneController() override;

    void step() override;

    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();

    void updateFitness(double newFitness);

    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

    std::vector<double> getWeights() const;

    std::vector<double> getInputs();

    int getSplit();

protected:
    MaxOneWorldModel *m_wm;

    std::vector<double> weights;



};


#endif //ROBOROBO3_MAXONECONTROLLER_H
