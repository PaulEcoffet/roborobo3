//
// Created by paul on 27/10/17.
//

#include "neuralnetworks/Perceptron.h"
#include "Utilities/Misc.h"
#include "WorldModels/RobotWorldModel.h"
#include "Agents/Agent.h"
#include "RoboroboMain/main.h"
#include <set>
#include <PartnerControl/include/PartnerControlWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "PartnerControl/include/PartnerControlController.h"
#include "PartnerControl/include/PartnerControlSharedData.h"

enum {
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

PartnerControlController::PartnerControlController(RobotWorldModel* wm)
{
    m_wm = dynamic_cast<PartnerControlWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbInputs = getNbInputs();
    unsigned int nbOutputs = getNbOutputs();

    switch (PartnerControlSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            std::cout << "MLP\n";
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(weights, nbInputs, nbOutputs);
            break;
        case ELMAN_ID:
            m_nn = new Elman(weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            std::cout << "Elman\n";
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "<< PartnerControlSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);


    m_nn->setWeights(weights);
    resetFitness();
}

void PartnerControlController::reset()
{
    if (PartnerControlSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman*>(m_nn)->initLastOutputs();
}

void PartnerControlController::step()
{
    if (not m_wm->isAlive())
        return;

    std::vector<double> inputs = getInputs();

    m_nn->setInputs(inputs);
    m_nn->step();
    std::vector<double> outputs = m_nn->readOut();

    if (m_wm->onOpportunity) // Bump in the coop opportunity so you cannot move
    {
        m_wm->_desiredTranslationalValue = gMaxTranslationalSpeed;
        m_wm->_desiredRotationalVelocity = 0;
    }
    else
    {
        m_wm->_desiredTranslationalValue = (outputs[0] + 1.0) / 2.0 * gMaxTranslationalSpeed;
        m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;
    }

    m_wm->_cooperationLevel = outputs[2] + 1; // Range between [0; 2]
}

std::vector<unsigned int> PartnerControlController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(PartnerControlSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(PartnerControlSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


PartnerControlController::~PartnerControlController()
{
    delete m_nn;
}

std::vector<double> PartnerControlController::getInputs()
{
    const int WALL_ID = 0;
    std::vector<double> inputs;
    inputs.reserve(m_nn->getNbInputs());

    /*
     * Camera inputs
     */
    for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
    {
        bool isOpportunity = false;
        double seenCoop = 0;
        auto entityId = static_cast<int>(m_wm->getObjectIdFromCameraSensor(i));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<PartnerControlOpportunity *>(gPhysicalObjects[entityId -
                                                                                           gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (PartnerControlSharedData::seeCoopFromDist)
                seenCoop = opportunity->getCoop();
            else
                seenCoop = 0;
        }
        inputs.emplace_back(m_wm->getDistanceValueFromCameraSensor(i) / m_wm->getCameraSensorMaximumDistanceValue(i));
        inputs.emplace_back(static_cast<double> (Agent::isInstanceOf(entityId)));
        inputs.emplace_back(static_cast<double> (entityId == WALL_ID));
        inputs.emplace_back(static_cast<double> (isOpportunity));
        inputs.emplace_back(seenCoop);
    }

    /*
     * Opportunity inputs
     */
    inputs.emplace_back(static_cast<double>(m_wm->onOpportunity));
    inputs.emplace_back(m_wm->meanLastTotalInvest());
    inputs.emplace_back(m_wm->meanLastOwnInvest());

    return inputs;
}

void PartnerControlController::loadNewGenome(const std::vector<double> &newGenome)
{
    if(m_nn->getRequiredNumberOfWeights() != newGenome.size())
    {
        std::cout << m_nn->getRequiredNumberOfWeights() << "!=" << newGenome.size() << std::endl;
        exit(-1);
    }
    weights = newGenome;
    m_nn->setWeights(weights);
    if (PartnerControlSharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman*>(m_nn)->initLastOutputs();
}

unsigned int PartnerControlController::getNbInputs() const
{
    return static_cast<unsigned int>(
            m_wm->_cameraSensorsNb * 5 // dist + isWall + isRobot + isObj + nbRob
            + 1 // onOpportunity
            + 1 // avgTotalEffort
            + 1 // avgEffort
    );
}


double PartnerControlController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void PartnerControlController::resetFitness()
{
    updateFitness(0);
}

void PartnerControlController::updateFitness( double newFitness )
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void PartnerControlController::increaseFitness( double delta )
{
    updateFitness(m_wm->_fitnessValue+delta);
}

std::string PartnerControlController::inspect(std::string prefix)
{
    std::stringstream out;

    std::set<int> seen;
    for (int i = 0; i < m_wm->_cameraSensorsNb; i++)
    {
        seen.insert((int) m_wm->getObjectIdFromCameraSensor(i));
    }

    out << "Seen objects:\n";
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
            auto coop = dynamic_cast<PartnerControlOpportunity *>(gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            out << "with " << coop->getNbNearbyRobots() << " robots nearby.\n ";
        }
    }
    if (m_wm->onOpportunity)
    {
        out << "On opportunity\n";
        out << "\tLast own invest: ";
        for (auto ownInvest : m_wm->lastOwnInvest)
        {
            out << ownInvest << " ";
        }
        out << "(" << m_wm->meanLastOwnInvest() << ")";
        out << "\n";
        out << "\tLast total invest: ";
        for (auto totInvest : m_wm->lastTotalInvest)
        {
            out << totInvest << " ";
        }
        out << "(" << m_wm->meanLastTotalInvest() << ")";
        out << "\n";

    }
    out << "Actual fitness: " << getFitness() << "\n";
    return out.str();
}

std::vector<double> PartnerControlController::getWeights() const
{
    return weights;
}

unsigned int PartnerControlController::getNbOutputs() const
{
    return 2    // Motor commands
           + 1  // Cooperation value
    ;
}


