/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_LIONCONTROLLER_H
#define ROBOROBO3_LIONCONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "LionOpportunity.h"
#include "LionWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the Lion experiment. Very similar to the LionController.
 */

class LionController : public Controller
{
public:
    explicit LionController(RobotWorldModel *wm);

    ~LionController() override;

    void step() override;

    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();

    void updateFitness(double newFitness);

    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

    const std::vector<double> getWeights() const;


    std::vector<double> getInputs();

protected:
    LionWorldModel *m_wm;

    NeuralNetwork *m_nn;
    NeuralNetwork *m_nn2;

    std::vector<double> weights;
    std::vector<double> weights2;


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    void fillNames();

    unsigned int getNbInputs() const;

    unsigned int getNbOutputs() const;

    static std::vector<std::string> inputnames;

    std::vector<double> getGameInputs() const;

    unsigned int getNbCameraInputs() const;

    unsigned int getNbGameInputs() const;

    std::vector<double> getCameraInputs() const;

    bool fill_names = false;

    int verbose = 0;
    double hardcoop;
};


#endif //ROBOROBO3_LIONCONTROLLER_H
