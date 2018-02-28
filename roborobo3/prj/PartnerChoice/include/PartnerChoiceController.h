/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_PARTNERCHOICECONTROLLER_H
#define ROBOROBO3_PARTNERCHOICECONTROLLER_H

#include <vector>
#include <contrib/network/PyevoInterface.h>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "PartnerChoiceOpportunity.h"
#include "PartnerChoiceWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the PartnerChoice experiment. Very similar to the CoopFixed2Controller.
 */

class PartnerChoiceController : public Controller
{
public:
    explicit PartnerChoiceController(RobotWorldModel *wm);
    ~PartnerChoiceController() override;

    void step() override;
    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);
    unsigned long getGenomeSize() const;

    void resetFitness();
    void updateFitness(double newFitness);
    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

protected:
    PartnerChoiceWorldModel *m_wm;

    NeuralNetwork *m_nn;
    std::vector<double> m_weights;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;

};


#endif //ROBOROBO3_PARTNERCHOICECONTROLLER_H
