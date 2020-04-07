/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-27
*/

#ifndef ROBOROBO3_SKILLEDCONTROLLER_H
#define ROBOROBO3_SKILLEDCONTROLLER_H

#include <vector>
#include "Controllers/Controller.h"
#include "SkilledWorldObserver.h"
#include "neuralnetworks/NeuralNetwork.h"
#include "SkilledOpportunity.h"
#include "SkilledWorldModel.h"
#include "Utilities/Misc.h"
#include "SkilledScoreLogger.h"


class SkilledWorldObserver;


using namespace Neural;

/**
 * Controller for the Skilled experiment. Very similar to the SkilledController.
 */

class SkilledController : public Controller {
public:
    explicit SkilledController(RobotWorldModel *wm);

    ~SkilledController() override;

    void step() override;

    void reset() override;

    void loadNewGenome(const std::vector<double> &newGenome);

    void resetFitness();

    void updateFitness(double newFitness);

    void increaseFitness(double delta);

    std::string inspect(std::string prefix) override;

    double getFitness() const;

    std::vector<double> getWeights() const;


    double getCoop(int i);

    double computeScore(int cost, int nbPart, double owncoop, double totothercoop);

    double getCoopWeight();

    void play_and_fitness();

protected:
    SkilledWorldModel *m_wm;
    SkilledWorldObserver *m_wo;

    NeuralNetwork *m_nn;

    std::vector<double> weights;
    std::vector<double> weights2;

    std::vector<unsigned int> getNbNeuronsPerHiddenLayers() const;

    SkilledScoreLogger *scorelogger;
    int verbose = 0;

    double cachedEmptyOpp = -1;


    unsigned int getNbInputs() const;

    unsigned int getNbOutputs() const;


    double computeScoreFromOpp(SkilledOpportunity *testopp, bool log);

    void move();
};


#endif //ROBOROBO3_SKILLEDCONTROLLER_H
