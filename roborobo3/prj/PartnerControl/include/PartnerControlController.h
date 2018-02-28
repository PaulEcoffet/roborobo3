/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_PARTNERCONTROLCONTROLLER_H
#define ROBOROBO3_PARTNERCONTROLCONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "PartnerControlOpportunity.h"
#include "PartnerControlWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the PartnerControl experiment. Very similar to the CoopFixed2Controller.
 */

class PartnerControlController : public Controller
{
public:
    explicit PartnerControlController(RobotWorldModel *wm);
    ~PartnerControlController() override;

    void step() override;
    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();
    void updateFitness(double newFitness);
    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;
    std::vector<double> getWeights() const;

protected:
    PartnerControlWorldModel *m_wm;

    NeuralNetwork *m_nn;
    std::vector<double> weights;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;
};


#endif //ROBOROBO3_PARTNERCONTROLCONTROLLER_H
