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
#include "core/Utilities/Misc.h"

using namespace Neural;

/**
 * Controller for the PartnerControl experiment. Very similar to the CoopFixed2Controller.
 */

class PartnerControlController : public Controller
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
                if (newVal < PartnerControlController::minWeight)
                {
                    const double range = PartnerControlController::maxWeight - PartnerControlController::minWeight;
                    double overflow = computeModulo(PartnerControlController::minWeight - newVal, range);
                    newVal = PartnerControlController::minWeight + overflow;
                }
                else if (newVal > PartnerControlController::maxWeight)
                {
                    const double range = PartnerControlController::maxWeight - PartnerControlController::minWeight;
                    double overflow = computeModulo(newVal - PartnerControlController::maxWeight, range);
                    newVal = PartnerControlController::maxWeight - overflow;
                }
                assert(PartnerControlController::minWeight <= newVal && newVal <= PartnerControlController::maxWeight);
                child.weights.push_back(newVal);
            }
            return child;
        }

    } genome;

    explicit PartnerControlController(RobotWorldModel *wm);
    ~PartnerControlController() override;

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
    PartnerControlWorldModel *m_wm;

    NeuralNetwork *m_nn;
    genome m_genome;

    std::vector<double> getInputs();


    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    unsigned int getNbInputs() const;
    unsigned int getNbOutputs() const;

    constexpr static double minWeight = -1;
    constexpr static double maxWeight = 1;
};


#endif //ROBOROBO3_PARTNERCONTROLCONTROLLER_H
