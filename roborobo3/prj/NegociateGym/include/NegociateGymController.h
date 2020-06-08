/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_NEGOCIATEGYMCONTROLLER_H
#define ROBOROBO3_NEGOCIATEGYMCONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "NegociateGymOpportunity.h"
#include "NegociateGymWorldModel.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the NegociateGym experiment. Very similar to the NegociateGymController.
 */

class NegociateGymController : public Controller
{
public:
    explicit NegociateGymController(RobotWorldModel *wm);

    ~NegociateGymController() override;

    void step() override;

    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();

    void updateFitness(double newFitness);

    void increaseFitness(double delta);

    bool acceptPlay();

    std::string inspect(std::string prefix) override;

    double getFitness() const;

    std::vector<double> getWeights() const;


    int getSplit();

protected:
    NegociateGymWorldModel *m_wm;

    NeuralNetwork *m_nn;
    NeuralNetwork *m_nn2;

    std::vector<double> weights;
    std::vector<double> weights2;


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    void fillNames();

    unsigned int getNbInputs() const;

    unsigned int getNbOutputs() const;

    std::vector<double> getGameInputs() const;

    unsigned int getNbCameraInputs() const;

    unsigned int getNbGameInputs() const;

    bool fill_names = false;

    int verbose = 0;
    double hardcoop;

    void wander_behavior() const;

    void seeking_behavior() const;
};


#endif //ROBOROBO3_NEGOCIATEGYMCONTROLLER_H
