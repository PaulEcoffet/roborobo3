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
#include <Negociate/include/NegociateWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "Negociate/include/NegociateController.h"
#include "Negociate/include/NegociateSharedData.h"

enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

std::vector<std::string> NegociateController::inputnames;

NegociateController::NegociateController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<NegociateWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbCamInputs = getNbCameraInputs();
    unsigned int nbGameInputs = getNbGameInputs();

    unsigned int nbMoveOutput = 1 + (int) !NegociateSharedData::tpToNewObj;
    unsigned int nbGameOutput = 1;

    fill_names = true;
    std::vector<unsigned int> nbNeurons2 = {3}; // TODO 3 should not be hardcode

    fillNames();
    switch (NegociateSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(weights, nbCamInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
            m_nn2 = new MLP(weights2, nbGameInputs, nbGameOutput, nbNeurons2, true);
            break;
        case PERCEPTRON_ID:
            if (NegociateSharedData::splitNetwork)
            {
                m_nn = new Perceptron(weights, nbCamInputs, nbMoveOutput);
                m_nn2 = new Perceptron(weights2, nbGameOutput, nbGameOutput);
            }
            else if (NegociateSharedData::onlyNforGame)
            {
                throw std::string("Not implemented");
            }
            else
            {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput);
            }
            break;
        case ELMAN_ID:
            if (NegociateSharedData::splitNetwork)
            {
                m_nn = new Elman(weights, nbCamInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new Elman(weights2, nbGameOutput, nbGameOutput, 2, true);
            }
            else if (NegociateSharedData::onlyNforGame)
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
                      << NegociateSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);


    weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
    m_nn2->setWeights(weights);

    resetFitness();
}

void NegociateController::reset()
{
    if (NegociateSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

void NegociateController::step()
{
    verbose = 0;
    m_wm->_cooperationLevel = hardcoop; // Range between [0; maxCoop]

    if (not m_wm->seeking)
    {
        if (NegociateSharedData::wander)
        {
            wander_behavior();
        }
    }
    else
    {
        seeking_behavior();
    }

}

void NegociateController::seeking_behavior() const
{
    std::vector<double> moveInputs = getCameraInputs();
    std::vector<double> gameInputs = getGameInputs();
    m_nn->setInputs(moveInputs);
    m_nn->step();

    /* Reading the output of the networks */
    std::vector<double> outputs = m_nn->readOut();


    if (NegociateSharedData::tpToNewObj)
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

void NegociateController::wander_behavior() const
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

std::vector<unsigned int> NegociateController::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(NegociateSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(NegociateSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


NegociateController::~NegociateController()
{
    delete m_nn;
}

std::vector<double> NegociateController::getInputs()
{
    std::vector<double> inputs;
    inputs = getCameraInputs();

    const std::vector<double> game_inputs(getGameInputs());
    inputs.insert(inputs.end(), game_inputs.begin(), game_inputs.end());
    fill_names = false;
    assert(inputnames.size() == inputs.size());
    return inputs;
}

std::vector<double> NegociateController::getCameraInputs() const
{
    size_t i = 0;
    std::vector<double> inputs(getNbCameraInputs(), 0);
    const int WALL_ID = 0;
    /*
     * Camera inputs
     */
    for (int j = 0; j < m_wm->_cameraSensorsNb; j++)
    {
        bool isOpportunity = false;
        double nbOnOpp = 0;
        double lastInvOnOpp = 0;
        double reputation = 0;
        int nbplays = 0;
        auto entityId = static_cast<int>(m_wm->getObjectIdFromCameraSensor(j));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<NegociateOpportunity *>(
                    gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            if (opportunity == m_wm->opp)
            {
                nbOnOpp = m_wm->nbOnOpp;
                lastInvOnOpp = m_wm->meanLastTotalInvest();
            }
            else
            {
                nbOnOpp = opportunity->getNbNearbyRobots();
                lastInvOnOpp = opportunity->curInv;
            }
        }
        else if (Agent::isInstanceOf(entityId))
        {
            reputation = m_wm->getOtherReputation(entityId - gRobotIndexStartOffset);
            nbplays = m_wm->getNbPlays(entityId - gRobotIndexStartOffset);
        }
        double dist = m_wm->getDistanceValueFromCameraSensor(j) / m_wm->getCameraSensorMaximumDistanceValue(j);
        inputs[i++] = (Agent::isInstanceOf(entityId)) ? dist : 1;
        if (NegociateSharedData::reputation)
        {
            inputs[i++] = reputation / NegociateSharedData::maxCoop;
            inputs[i++] = nbplays;
        }
        inputs[i++] = (entityId == WALL_ID) ? dist : 1;
        inputs[i++] = (isOpportunity) ? dist : 1;
        inputs[i++] = nbOnOpp;

        if (NegociateSharedData::reputation)
        {
            inputs[i++] = lastInvOnOpp / NegociateSharedData::maxCoop;
        }

    }

    return inputs;
}


void NegociateController::fillNames()
{
    if (inputnames.empty())
    {
        for (int j = 0; j < m_wm->_cameraSensorsNb; j++)
        {
            //inputnames.emplace_back("dist " + std::to_string(j));
            inputnames.emplace_back("dist robot");
            if (NegociateSharedData::reputation)
            {
                inputnames.emplace_back("reputation");
                inputnames.emplace_back("nb plays");
            }
            inputnames.emplace_back("dist wall");
            inputnames.emplace_back("dist obj");
            inputnames.emplace_back("nb on obj");
            if (NegociateSharedData::reputation)
            {
                inputnames.emplace_back("last inv on opp");
            }
        }

        inputnames.emplace_back("playing");
        if (NegociateSharedData::arrivalAsInput)
        {
            inputnames.emplace_back("arrival");
        }
        if (NegociateSharedData::totalInvAsInput)
        {
            inputnames.emplace_back("mean total inv");

        }
        if (NegociateSharedData::ownInvAsInput)
        {
            inputnames.emplace_back("mean own inv");
        }

        if (NegociateSharedData::punishmentAsInput)
        {
            inputnames.emplace_back("punishment");
        }

        /*
         * introspection inputs
         */
        if (NegociateSharedData::selfAAsInput)
        {
            inputnames.emplace_back("own A");
        }
        fill_names = false;
    }
}

std::vector<double> NegociateController::getGameInputs() const
{
    /*
     * Opportunity inputs
     */
    std::vector<double> inputs(getNbGameInputs(), 0);
    size_t i = 0;

    inputs[i++] = m_wm->isPlaying();
    if (NegociateSharedData::arrivalAsInput)
    {
        inputs[i++] = m_wm->arrival;
    }
    if (NegociateSharedData::totalInvAsInput)
    {
        inputs[i++] = m_wm->meanLastTotalInvest() / NegociateSharedData::maxCoop;
    }
    if (NegociateSharedData::ownInvAsInput)
    {
        inputs[i++] = m_wm->getCoop() / NegociateSharedData::maxCoop;
    }

    /*
     * introspection inputs
     */
    if (NegociateSharedData::selfAAsInput)
    {
        inputs[i++] = (m_wm->selfA - NegociateSharedData::meanA) / NegociateSharedData::stdA;
    }
    return inputs;
}

void NegociateController::loadNewGenome(const std::vector<double> &newGenome)
{
    int coopgene = 0;
    if (NegociateSharedData::fixCoop)
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

    if (NegociateSharedData::fixCoop)
    {
        hardcoop = newGenome[0] * NegociateSharedData::maxCoop;
        m_wm->_cooperationLevel = hardcoop;
    }
    if (NegociateSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

unsigned int NegociateController::getNbInputs() const
{
    return getNbCameraInputs() + getNbGameInputs();
}

unsigned int NegociateController::getNbGameInputs() const
{
    const auto nbGameInputs = static_cast<const unsigned int>(
            1
            + NegociateSharedData::arrivalAsInput
            + NegociateSharedData::totalInvAsInput
            + NegociateSharedData::ownInvAsInput
            + NegociateSharedData::selfAAsInput
    );
    return nbGameInputs;
}

unsigned int NegociateController::getNbCameraInputs() const
{
    const unsigned int nbCameraInputs = static_cast<const unsigned int>(
            m_wm->_cameraSensorsNb * (4 + 3 *
                                          (int) NegociateSharedData::reputation)); // dist + isWall + isRobot + isObj + nbRob + repopp + repAgent + nbplays
    return nbCameraInputs;
}


double NegociateController::getFitness() const
{
    return m_wm->_fitnessValue;
}


void NegociateController::resetFitness()
{
    updateFitness(0);
}

void NegociateController::updateFitness(double newFitness)
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void NegociateController::increaseFitness(double delta)
{
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string NegociateController::inspect(std::string prefix)
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
                auto coop = dynamic_cast<NegociateOpportunity *>(gPhysicalObjects[entityId -
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
        auto inputs = getInputs();
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

std::vector<double> NegociateController::getWeights() const
{
    std::vector<double> allweights;
    if (NegociateSharedData::fixCoop)
    {
        allweights.push_back(hardcoop);
    }
    allweights.insert(allweights.end(), weights.begin(), weights.end());

    allweights.insert(allweights.end(), weights2.begin(), weights2.end());

    return allweights;
}

unsigned int NegociateController::getNbOutputs() const
{
    return 1
           + (unsigned int) !NegociateSharedData::tpToNewObj // Motor commands
           + (unsigned int) !NegociateSharedData::fixCoop  // Cooperation value
           + (unsigned int) NegociateSharedData::punishment;
}


bool NegociateController::acceptPlay()
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

int NegociateController::getSplit()
{
    return (int) NegociateSharedData::fixCoop + m_nn->getRequiredNumberOfWeights();
}
