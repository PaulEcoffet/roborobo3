//
// Created by paul on 27/10/17.
//

#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "World/World.h"
#include "RoboroboMain/main.h"
#include <set>
#include <NegociateGym/include/NegociateGymWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "NegociateGym/include/NegociateGymController.h"
#include "NegociateGym/include/NegociateGymSharedData.h"
#include "NegociateGymWorldModel.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

NegociateGymController::NegociateGymController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<NegociateGymWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbCamInputs = getNbCameraInputs();
    unsigned int nbGameInputs = getNbGameInputs();

    unsigned int nbMoveOutput = 1 + (int) !NegociateGymSharedData::tpToNewObj;
    unsigned int nbGameOutput = 1;

    fill_names = true;
    std::vector<unsigned int> nbNeurons2 = {3}; // TODO 3 should not be hardcode

    fillNames();
    switch (NegociateGymSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(weights, nbCamInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
            m_nn2 = new MLP(weights2, nbGameInputs, nbGameOutput, nbNeurons2, true);
            break;
        case PERCEPTRON_ID:
            if (NegociateGymSharedData::splitNetwork)
            {
                m_nn = new Perceptron(weights, nbCamInputs, nbMoveOutput);
                m_nn2 = new Perceptron(weights2, nbGameOutput, nbGameOutput);
            }
            else if (NegociateGymSharedData::onlyNforGame)
            {
                throw std::string("Not implemented");
            }
            else
            {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput);
            }
            break;
        case ELMAN_ID:
            if (NegociateGymSharedData::splitNetwork)
            {
                m_nn = new Elman(weights, nbCamInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new Elman(weights2, nbGameOutput, nbGameOutput, 2, true);
            }
            else if (NegociateGymSharedData::onlyNforGame)
            {
                throw std::string("Not implemented");
            }
            else
            {
                m_nn = new Elman(weights, nbCamInputs, nbMoveOutput + nbGameOutput,
                                 nbNeuronsPerHiddenLayers, true);
            }
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << NegociateGymSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);


    weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
    m_nn2->setWeights(weights);

    std::cout << "number of weights nn1: " << m_nn->getRequiredNumberOfWeights() << std::endl;
    std::cout << "number of weights nn2: " << m_nn2->getRequiredNumberOfWeights() << std::endl;

    resetFitness();
}

void NegociateGymController::reset()
{
    if (NegociateGymSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

void NegociateGymController::step()
{
    verbose = 0;
    m_wm->_cooperationLevel = hardcoop; // Range between [0; maxCoop]

    if (not m_wm->seeking)
    {
        if (NegociateGymSharedData::wander)
        {
            wander_behavior();
        }
    }
    else
    {
        seeking_behavior();
    }

}

void NegociateGymController::seeking_behavior() const
{
    std::vector<double> moveInputs = m_wm->getCameraInputs();
    std::vector<double> gameInputs = m_wm->getGameInputs();
    m_nn->setInputs(moveInputs);
    m_nn->step();

    /* Reading the output of the networks */
    std::vector<double> outputs = m_nn->readOut();


    if (NegociateGymSharedData::tpToNewObj)
    {
        m_wm->_desiredTranslationalValue = 0;
        m_wm->_desiredRotationalVelocity = 0;
        m_wm->teleport = outputs[0] > 0;
    }
    else
    {
        m_wm->_desiredTranslationalValue = outputs[0] * gMaxTranslationalSpeed;
        m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;
    }
}

void NegociateGymController::wander_behavior() const
{
    const double sensorlength = 30; // pixels
    double meandistfront = (
                                   m_wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) +
                                   m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE)
                           ) / 2;
    if (meandistfront > sensorlength)
    {
        meandistfront = sensorlength;
    }
    m_wm->_desiredTranslationalValue = +2 - 2 * ((sensorlength - meandistfront) / sensorlength);

    if (m_wm->getCameraSensorValue(0, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(1, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(2, SENSOR_DISTANCEVALUE) <
        m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
        m_wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE))
    {
        m_wm->_desiredRotationalVelocity = +10;
    }
    else if (m_wm->getCameraSensorValue(3, SENSOR_DISTANCEVALUE) +
             m_wm->getCameraSensorValue(4, SENSOR_DISTANCEVALUE) +
             m_wm->getCameraSensorValue(5, SENSOR_DISTANCEVALUE) < 3 * gSensorRange)
    {
        m_wm->_desiredRotationalVelocity = -10;
    }
    else if (m_wm->_desiredRotationalVelocity > 0)
    {
        m_wm->_desiredRotationalVelocity--;
    }
    else if (m_wm->_desiredRotationalVelocity < 0)
    {
        m_wm->_desiredRotationalVelocity++;
    }
    else
    {
        m_wm->_desiredRotationalVelocity = 0.01 - (double) (randint() % 10) / 10. * 0.02;
    }
}

std::vector<unsigned int> NegociateGymController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(NegociateGymSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(NegociateGymSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


NegociateGymController::~NegociateGymController()
{
    delete m_nn;
}

std::vector<double> NegociateGymController::getGameInputs() const
{
    /*
     * Opportunity inputs
     */
    std::vector<double> inputs(getNbGameInputs(), 0);
    size_t i = 0;

    inputs[i++] = m_wm->isPlaying();
    if (NegociateGymSharedData::arrivalAsInput)
    {
        inputs[i++] = m_wm->arrival;
    }
    if (NegociateGymSharedData::totalInvAsInput)
    {
        inputs[i++] = m_wm->meanLastTotalInvest() / NegociateGymSharedData::maxCoop;
    }
    if (NegociateGymSharedData::ownInvAsInput)
    {
        inputs[i++] = m_wm->getCoop() / NegociateGymSharedData::maxCoop;
    }

    /*
     * introspection inputs
     */
    if (NegociateGymSharedData::selfAAsInput)
    {
        inputs[i++] = (m_wm->selfA - NegociateGymSharedData::meanA) / NegociateGymSharedData::stdA;
    }
    return inputs;
}

void NegociateGymController::loadNewGenome(const std::vector<double> &newGenome)
{
    int coopgene = 0;
    if (NegociateGymSharedData::fixCoop)
    {
        coopgene = 1;
    }

    if (coopgene + m_nn->getRequiredNumberOfWeights() + m_nn2->getRequiredNumberOfWeights() != newGenome.size())
    {
        std::cout << "nb weights does not match nb genes: " << m_nn->getRequiredNumberOfWeights() << "!="
                  << newGenome.size() << std::endl;
        exit(-1);
    }
    auto split = newGenome.begin() + m_nn->getRequiredNumberOfWeights() + coopgene;
    weights = std::vector<double>(newGenome.begin() + coopgene, split);
    weights2 = std::vector<double>(split, newGenome.end());
    m_nn->setWeights(weights);
    m_nn2->setWeights(weights2);

    if (NegociateGymSharedData::fixCoop)
    {
        hardcoop = newGenome[0] * NegociateGymSharedData::maxCoop;
        m_wm->_cooperationLevel = hardcoop;
    }
    if (NegociateGymSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

unsigned int NegociateGymController::getNbInputs() const
{
    return getNbCameraInputs() + getNbGameInputs();
}

unsigned int NegociateGymController::getNbGameInputs() const
{
    const auto nbGameInputs = static_cast<const unsigned int>(
            1
            + NegociateGymSharedData::arrivalAsInput
            + NegociateGymSharedData::totalInvAsInput
            + NegociateGymSharedData::ownInvAsInput
            + NegociateGymSharedData::selfAAsInput
    );
    return nbGameInputs;
}

unsigned int NegociateGymController::getNbCameraInputs() const
{
    const unsigned int nbCameraInputs = static_cast<const unsigned int>(
            m_wm->_cameraSensorsNb * (4 + 3 *
                                          (int) NegociateGymSharedData::reputation)); // dist + isWall + isRobot + isObj + nbRob + repopp + repAgent + nbplays
    return nbCameraInputs;
}


double NegociateGymController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void NegociateGymController::resetFitness()
{
    updateFitness(0);
}

void NegociateGymController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void NegociateGymController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string NegociateGymController::inspect(std::string prefix)
{
    std::stringstream out;
    if (verbose == 0)
    {
        out << prefix << "I'm robot with coop coef " << m_wm->fakeCoef << "\n";
        std::set<int> seen;
        for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
        {
            seen.insert((int) m_wm->getObjectIdFromCameraSensor(i));
        }

        out << prefix << "Seen objects:\n";
        for (int entityId : seen)
        {
            if (entityId == 0)
            {
                out << "\tA wall\n";
            }
            else if (Agent::isInstanceOf(entityId))
            {
                out << "\tAnother agent\n";
            }
            else if (entityId >= gPhysicalObjectIndexStartOffset)
            {
                out << "\tA cooperation opportunity ";
                auto coop = dynamic_cast<NegociateGymOpportunity *>(gPhysicalObjects[entityId -
                                                                                  gPhysicalObjectIndexStartOffset]);
                out << "with " << coop->getNbNearbyRobots() << " robots nearby.\n ";
            }
        }
        if (m_wm->onOpportunity)
        {
            out << prefix << "On opportunity with " << m_wm->nbOnOpp << ". I arrived " << m_wm->arrival << ".\n";
            out << prefix << "\tLast own invest: ";
            for (auto ownInvest : m_wm->lastOwnInvest)
            {
                out << ownInvest << " ";
            }
            out << "(mean : " << m_wm->meanLastOwnInvest() << ")";
            out << "\n";
            out << prefix << "\tLast total invest: ";
            for (auto totInvest : m_wm->lastTotalInvest)
            {
                out << totInvest << " ";
            }
            out << "(mean : " << m_wm->meanLastTotalInvest() << ")";
            out << "\n";

        }
        out << prefix << "a coeff: " << m_wm->selfA << "\n";
        out << prefix << "last coop: " << m_wm->getCoop() << " (true val: " << m_wm->getCoop(true) << ")\n";
        out << prefix << "reputation : " << m_wm->meanLastCommonKnowledgeReputation() << "\n";
        out << prefix << "received punishment : " << m_wm->punishment << "\n";
        out << prefix << "sent punishment : " << m_wm->spite << "\n";

        out << prefix << "Actual fitness: " << getFitness() << "\n";
    }
    if (verbose == 1)
    {
        auto inputs = m_wm->getInputs();
        out << prefix << "inputs:\n";
        for (size_t i = 0; i < inputs.size(); i++)
        {
            out << prefix << "\t" << inputnames[i] << ":" << inputs[i] << "\n";
        }
        out << prefix << "outputs:\n";
        auto &outputs = m_nn->readOut();
        for (auto &output : outputs)
        {
            out << prefix << "\t" << output << "\n"; // TODO Crash without reason
        }

        auto &outputs2 = m_nn2->readOut();
        for (auto &output : outputs2)
        {
            out << prefix << "\t" << output << "\n"; // TODO Crash without reason
        }

    }
    if (verbose == 2)
    {
        out << m_nn->toString() << std::endl;
        out << m_nn2->toString() << std::endl;

    }
    verbose++;
    return out.str();
}

std::vector<double> NegociateGymController::getWeights() const
{
    std::vector<double> allweights;
    if (NegociateGymSharedData::fixCoop)
    {
        allweights.push_back(hardcoop);
    }
    allweights.insert(allweights.end(), weights.begin(), weights.end());

    allweights.insert(allweights.end(), weights2.begin(), weights2.end());

    return allweights;
}

unsigned int NegociateGymController::getNbOutputs() const
{
    return 1
           + (unsigned int) !NegociateGymSharedData::tpToNewObj // Motor commands
           + (unsigned int) !NegociateGymSharedData::fixCoop  // Cooperation value
           + (unsigned int) NegociateGymSharedData::punishment;
}


bool NegociateGymController::acceptPlay()
{
    auto input = getGameInputs();
    m_nn2->setInputs(input);
    m_nn2->step();
    auto output = m_nn2->readOut();
    /*
    std::cout << "My inputs: ";
    for (auto in : input)
    {
        std::cout << in << ",";
    }
    std::cout << ": " << (output[0] > 0) << "\n";
     */
    return (output[0] > 0);
}

int NegociateGymController::getSplit()
{
    return (int) NegociateGymSharedData::fixCoop + m_nn->getRequiredNumberOfWeights();
}
