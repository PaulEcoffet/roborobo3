/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_PARTNERCHOICECONTROLLER_H
#define ROBOROBO3_PARTNERCHOICECONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "PartnerChoiceOpportunity.h"

using namespace Neural;

/**
 * Controller for the PartnerChoice experiment. Very similar to the CoopFixed2Controller.
 */

class PartnerChoiceController : public Controller
{
public:
    typedef struct genome { std::vector<double> weights; double sigma;} genome;

    explicit PartnerChoiceController(RobotWorldModel *wm);
    ~PartnerChoiceController() override;

    void step() override;
    void reset() override;

    void loadNewGenome(const genome &newGenome);
    void mutateGenome();

    void resetFitness();
    void updateFitness(double newFitness);
    void increaseFitness(double delta);

    std::string inspect() override;

    double getFitness() const;
    genome getGenome() const;

protected:
    PartnerChoiceWorldModel *m_wm;

    NeuralNetwork *m_nn;
    genome m_genome;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;

    constexpr static double minWeight = -1;
    constexpr static double maxWeight = 1;
};


#endif //ROBOROBO3_PARTNERCHOICECONTROLLER_H
