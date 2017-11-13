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
#include "PartnerChoiceWorldModel.h"
#include "core/Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the PartnerChoice experiment. Very similar to the CoopFixed2Controller.
 */

class PartnerChoiceController : public Controller
{
public:
    typedef struct genome {
        std::vector<double> weights;
        double sigma;
        genome mutate()
        {
            genome child{};
            child.sigma = sigma;
            child.weights.reserve(weights.size());
            for (const double &weight : weights)
            {
                // Bouncing random
                double newVal = getGaussianRand(weight, sigma);
                if (newVal < PartnerChoiceController::minWeight)
                {
                    const double range = PartnerChoiceController::maxWeight - PartnerChoiceController::minWeight;
                    double overflow = computeModulo(PartnerChoiceController::minWeight - newVal, range);
                    newVal = PartnerChoiceController::minWeight + overflow;
                }
                else if (newVal > PartnerChoiceController::maxWeight)
                {
                    const double range = PartnerChoiceController::maxWeight - PartnerChoiceController::minWeight;
                    double overflow = computeModulo(newVal - PartnerChoiceController::maxWeight, range);
                    newVal = PartnerChoiceController::maxWeight - overflow;
                }
                assert(PartnerChoiceController::minWeight <= newVal && newVal <= PartnerChoiceController::maxWeight);
                child.weights.push_back(newVal);
            }
            return child;
        }

    } genome;

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
