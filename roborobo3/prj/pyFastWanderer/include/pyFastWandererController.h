/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_PYFASTWANDERERCONTROLLER_H
#define ROBOROBO3_PYFASTWANDERERCONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "pyFastWandererOpportunity.h"
#include "Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the pyFastWanderer experiment.
 */

class pyFastWandererController : public Controller
{
public:
    explicit pyFastWandererController(RobotWorldModel *wm);
    ~pyFastWandererController() override;

    void step() override;
    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();
    void updateFitness(double newFitness);
    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

    const std::vector<double>& getGenome() const;

protected:
    RobotWorldModel *m_wm;

    NeuralNetwork *m_nn;
    std::vector<double> m_genome;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;
};


#endif //ROBOROBO3_PYFASTWANDERERCONTROLLER_H
