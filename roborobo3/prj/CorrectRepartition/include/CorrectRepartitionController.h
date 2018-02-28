/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_CORRECTREPARTITIONCONTROLLER_H
#define ROBOROBO3_CORRECTREPARTITIONCONTROLLER_H

#include <vector>
#include <contrib/network/PyevoInterface.h>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "CorrectRepartitionOpportunity.h"
#include "CorrectRepartitionWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the CorrectRepartition experiment. Very similar to the CoopFixed2Controller.
 */

class CorrectRepartitionController : public Controller
{
public:
    explicit CorrectRepartitionController(RobotWorldModel *wm);
    ~CorrectRepartitionController() override;

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
    CorrectRepartitionWorldModel *m_wm;

    NeuralNetwork *m_nn;
    std::vector<double> m_weights;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;

};


#endif //ROBOROBO3_CORRECTREPARTITIONCONTROLLER_H
