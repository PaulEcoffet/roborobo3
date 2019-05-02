//
// Created by paul on 27/10/17.
//

#include <contrib/neuralnetworks/EigenMLP.h>
#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "World/World.h"
#include "RoboroboMain/main.h"
#include <set>
#include <Lion/include/LionWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "Lion/include/LionController.h"
#include "Lion/include/LionSharedData.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

LionController::LionController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<LionWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    std::vector<unsigned int> nbNeurons2 = std::vector<unsigned int>(1, 2);
    switch (LionSharedData::controllerType)
    {
        case MLP_ID:
                m_nn = new MLP(weights, 4, 1,
                               nbNeuronsPerHiddenLayers, true);
                m_nn2 = new MLP(weights, 1, 1, nbNeurons2, true);

            break;
        case PERCEPTRON_ID:
                m_nn = new Perceptron(weights, 4, 1);
                m_nn2 = new Perceptron(weights, 1, 1);
            break;
        case ELMAN_ID:
                m_nn = new Elman(weights, 4, 1,
                                 nbNeuronsPerHiddenLayers, true);
                m_nn2 = new Elman(weights, 4, 1,
                                 nbNeurons2, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << LionSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);
    weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
    m_nn2->setWeights(weights2);

    resetFitness();
}

void LionController::reset()
{
    if (LionSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

void LionController::step()
{
    verbose = 0;

    if (not m_wm->isAlive())
    {
        return;
    }

    LionOpportunity *best = nullptr;
    double bestscore = -9999;

    int curoppid = (m_wm->opp) ? m_wm->opp->getId() : -1;
    std::vector<double> inputs(4, 0);
    int i = 0;

    for (auto *opp : gPhysicalObjects)
    {
        auto *lionopp = dynamic_cast<LionOpportunity *>(opp);
        double cost = opp->getId() != curoppid;
        double onopp = 1 - cost;
        double nbopp = lionopp->countCurrentRobots() - onopp;
        double owncoop = m_wm->getCoop(nbopp);
        double coop = 0;
        if (onopp)
        {
            coop = (lionopp->getCurInv() - owncoop);
        }
        else
        {
            coop = lionopp->getIfNewPartInv();
        }
        i = 0;
        inputs[i++] = cost;
        inputs[i++] = nbopp / gNbOfRobots;
        inputs[i++] = coop / LionSharedData::maxCoop;
        inputs[i] = owncoop / LionSharedData::maxCoop;
        m_nn->setInputs(inputs);

        m_nn->step();
        std::vector<double> outputs = m_nn->readOut();
        if (m_wm->getId() == 0)
        {
            //std::cout << opp->getId() << ": cost:" << cost << ", nb:" << nbopp << ", coop:" << coop << ", own:" << owncoop << ", score :" << outputs[0] << std::endl;
        }
        if (outputs[0] > bestscore)
        {
            bestscore = outputs[0];
            best = lionopp;
        }
    }


    /* Reading the output of the networks */

    m_wm->_desiredTranslationalValue = 0;
    m_wm->_desiredRotationalVelocity = 0;
    m_wm->teleport = best->getId();

    if (m_wm->fakeCoef < 0.8)
    {
        m_wm->setRobotLED_colorValues(126, 55, 49);
        if (m_wm->onOpportunity)
        {
            m_wm->setRobotLED_colorValues(169, 96, 89);
        }
    }
    else if (m_wm->fakeCoef < 1.2)
    {
        m_wm->setRobotLED_colorValues(0, 0, 255);
        if (m_wm->onOpportunity)
        {
            m_wm->setRobotLED_colorValues(100, 100, 255);
        }
    }
    else
    {
        m_wm->setRobotLED_colorValues(115, 182, 234);
        if (m_wm->onOpportunity)
        {
            m_wm->setRobotLED_colorValues(185, 218, 244);
        }
    }
}

std::vector<unsigned int> LionController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(LionSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(LionSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


LionController::~LionController()
{
    delete m_nn;
}


void LionController::loadNewGenome(const std::vector<double> &newGenome)
{

    auto split = newGenome.begin() + m_nn->getRequiredNumberOfWeights();
    if (m_nn->getRequiredNumberOfWeights() + m_nn2->getRequiredNumberOfWeights() != newGenome.size())
    {
        std::cout << "nb weights does not match nb genes: " << m_nn->getRequiredNumberOfWeights() << "!="
                  << newGenome.size() << std::endl;
        exit(-1);
    }
    weights = std::vector<double>(newGenome.begin(), split);
    weights2 = std::vector<double>(split, newGenome.end());
    m_nn->setWeights(weights);
    m_nn2->setWeights(weights2);

    if (LionSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
    std::vector<double> inputs(1, 0);
    for(int i = 0; i < gNbOfRobots; i++)
    {
        inputs[0] = i;
        m_nn2->setInputs(inputs);
        m_nn2->step();
        auto output = m_nn2->readOut();
        m_wm->setCoop(i, (double)((output[0] + 1)) / 2.0 * LionSharedData::maxCoop);
    }
}

unsigned int LionController::getNbInputs() const
{
    return 3;
}


double LionController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void LionController::resetFitness()
{
    updateFitness(0);
}

void LionController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void LionController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string LionController::inspect(std::string prefix)
{
    std::stringstream out;
    if (verbose == 0)
    {
        out << prefix << "I'm robot with coop coef " << m_wm->fakeCoef << "\n";

        if (m_wm->opp)
        {
            out << prefix << "WARNING: THE DISPLAYED VALUES DO NOT CORRESPOND TO WHAT HAS REALLY BEEN PLAYED\n";
            out << prefix << "On opportunity with " << m_wm->opp->countCurrentRobots() - 1 << ".\n";
            out << prefix << "\tLast own invest: ";
            out <<  m_wm->getCoop(m_wm->opp->countCurrentRobots() - 1) << " (" << m_wm->getCoop(m_wm->opp->countCurrentRobots() - 1, true) << ")";
            out << "\n";
            out << prefix << "\tLast total invest: ";
            out << m_wm->opp->getCurInv();
            out << "\n";

        }
        out << prefix << "a coeff: " << m_wm->selfA << "\n";
        out << prefix << "Actual fitness: " << getFitness() << "\n";
    }
    if (verbose == 1)
    {
        out << m_nn->toString() << std::endl;
        out << m_nn2->toString() << std::endl;
    }
    verbose = (verbose + 1) % 2;
    return out.str();
}

const std::vector<double> LionController::getWeights() const
{
    std::vector<double> allweights;

    allweights.insert(allweights.end(), weights.begin(), weights.end());
    allweights.insert(allweights.end(), weights2.begin(), weights2.end());

    return allweights;
}

unsigned int LionController::getNbOutputs() const
{
    return 1;
}
