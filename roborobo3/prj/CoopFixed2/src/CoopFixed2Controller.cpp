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
#include <CoopFixed2/include/CoopFixed2WorldModel.h>
#include "neuralnetworks/Elman.h"
#include "CoopFixed2/include/CoopFixed2Controller.h"
#include "CoopFixed2/include/CoopFixed2SharedData.h"

enum {
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

std::vector<std::string> CoopFixed2Controller::inputnames;

CoopFixed2Controller::CoopFixed2Controller(RobotWorldModel* wm) {
    m_wm = dynamic_cast<CoopFixed2WorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbCamInputs = getNbCameraInputs();
    unsigned int nbGameInputs = getNbGameInputs();

    unsigned int nbMoveOutput = 2;
    unsigned int nbGameOutput = 1;

    switch (CoopFixed2SharedData::controllerType) {
        case MLP_ID:
            if (CoopFixed2SharedData::splitNetwork) {
                m_nn = new MLP(weights, nbCamInputs + nbGameInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new MLP(weights2, nbGameInputs, nbGameOutput, 2, true);  // TODO 2 should not be hardcoded
            } else {
                m_nn = new MLP(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput,
                               nbNeuronsPerHiddenLayers, true);
            }
            break;
        case PERCEPTRON_ID:
            if (CoopFixed2SharedData::splitNetwork) {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput);
                m_nn2 = new Perceptron(weights2, nbGameOutput, nbGameOutput);
            } else {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput);
            }
            break;
        case ELMAN_ID:
            if (CoopFixed2SharedData::splitNetwork) {
                m_nn = new Elman(weights, nbCamInputs + nbGameInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new Elman(weights2, nbGameOutput, nbGameOutput, 2, true);
            } else {
                m_nn = new Elman(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput, nbNeuronsPerHiddenLayers, true);
            }
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << CoopFixed2SharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);

    if (CoopFixed2SharedData::splitNetwork)
    {
        weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
        m_nn2->setWeights(weights);
    }

    resetFitness();
}

void CoopFixed2Controller::reset()
{
    if (CoopFixed2SharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman*>(m_nn)->initLastOutputs();
}

void CoopFixed2Controller::step()
{
    if (not m_wm->isAlive())
        return;

    fill_names = inputnames.empty();
    if (CoopFixed2SharedData::splitNetwork)
    {
        std::vector<double> moveInputs = getCameraInputs();
        std::vector<double> gameInputs = getGameInputs();
        std::vector<double> allInputs;
        allInputs.insert(allInputs.end(), moveInputs.begin(), moveInputs.end());
        allInputs.insert(allInputs.end(), gameInputs.begin(), gameInputs.end());
        m_nn->setInputs(allInputs);
        m_nn->step();

        m_nn2->setInputs(gameInputs);
        m_nn2->step();
    } else {
        std::vector<double> inputs = getInputs();
        m_nn->setInputs(inputs);
        m_nn->step();

    }

    /* Reading the output of the networks */
    std::vector<double> outputs = m_nn->readOut();

    if (CoopFixed2SharedData::splitNetwork)
    {
        auto gameoutput = m_nn2->readOut();
        outputs.insert(outputs.end(), gameoutput.begin(), gameoutput.end() );
    }
    assert(outputs.size() == getNbOutputs());
    if (CoopFixed2SharedData::tpToNewObj) {
        m_wm->_desiredTranslationalValue = 0;
        m_wm->_desiredRotationalVelocity = 0;
        m_wm->teleport = outputs[0] > 0;
    } else {
        m_wm->_desiredTranslationalValue = outputs[0] * gMaxTranslationalSpeed;
        m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;

    }
    double coop;
    if (CoopFixed2SharedData::reverseCoopOutput == false) {
        coop = ((outputs[2] + 1) / 2) * CoopFixed2SharedData::maxCoop;
    } else {
        coop = (1 - ((outputs[2] + 1) / 2)) * CoopFixed2SharedData::maxCoop;
    }
    m_wm->_cooperationLevel = coop; // Range between [0; maxCoop]

    if (m_wm->fake)
    {
        if (m_wm->fakeCoef < 1)
            m_wm->setRobotLED_colorValues(255, 0, 0);
        else
            m_wm->setRobotLED_colorValues(127, 255, 255);
    }
    else
    {
        if (m_wm->onOpportunity && (!CoopFixed2SharedData::fixRobotNb || m_wm->arrival <=2))
        {
            m_wm->setRobotLED_colorValues(0, 255, 0);
        }
        else
        {
            m_wm->setRobotLED_colorValues(0, 0, 255);
        }
    }

}

std::vector<unsigned int> CoopFixed2Controller::getNbNeuronsPerHiddenLayers() const
{
    auto nbHiddenLayers = static_cast<unsigned int>(CoopFixed2SharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer)
    {
        nbNeuro = static_cast<unsigned int>(CoopFixed2SharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


CoopFixed2Controller::~CoopFixed2Controller()
{
    delete m_nn;
}

std::vector<double> CoopFixed2Controller::getInputs()
{
    size_t i;
    std::vector<double> inputs;
    inputs = getCameraInputs();

    const std::vector<double> game_inputs(getGameInputs());
    inputs.insert(inputs.end(), game_inputs.begin(), game_inputs.end());


    assert((CoopFixed2SharedData::splitNetwork && inputs.size() == m_nn->getNbInputs() + m_nn2->getNbInputs()) || inputs.size() == m_nn->getNbInputs());
    assert(inputnames.size() == inputs.size());
    return inputs;
}

std::vector<double> CoopFixed2Controller::getCameraInputs() const {
    size_t i= 0;
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
        auto entityId = static_cast<int>(m_wm->getObjectIdFromCameraSensor(j));

        if (entityId >= gPhysicalObjectIndexStartOffset &&
            entityId < gPhysicalObjectIndexStartOffset + gNbOfPhysicalObjects) // is an Object
        {
            auto *opportunity = dynamic_cast<CoopFixed2Opportunity *>(
                    gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            isOpportunity = true;
            nbOnOpp = opportunity->getNbNearbyRobots();
            lastInvOnOpp = opportunity->curInv;
        }
        else if (Agent::isInstanceOf(entityId))
        {
            auto* rwm = dynamic_cast<CoopFixed2WorldModel*> (gWorld->getRobot(entityId - gRobotIndexStartOffset)->getWorldModel());
            reputation = rwm->meanLastReputation();
        }
        inputs[i++] = m_wm->getDistanceValueFromCameraSensor(j) / m_wm->getCameraSensorMaximumDistanceValue(j);
        inputs[i++] = static_cast<double> (Agent::isInstanceOf(entityId));
        if (CoopFixed2SharedData::reputation) {
            inputs[i++] = reputation;
        }
        inputs[i++] = static_cast<double> (entityId == WALL_ID);
        inputs[i++] = static_cast<double> (isOpportunity);
        inputs[i++] = nbOnOpp;

        if (CoopFixed2SharedData::reputation)
        {
            inputs[i++] = lastInvOnOpp;
        }
        if (fill_names) {
            inputnames.emplace_back("dist " + std::__cxx11::to_string(j));
            inputnames.emplace_back("is robot");
            if (CoopFixed2SharedData::reputation)
            {
                inputnames.emplace_back("reputation");
            }
            inputnames.emplace_back("is wall");
            inputnames.emplace_back("is obj");
            inputnames.emplace_back("nb on obj");
            if (CoopFixed2SharedData::reputation)
            {
                inputnames.emplace_back("last inv on opp");
            }
        }

    }

    return inputs;
}

std::vector<double> CoopFixed2Controller::getGameInputs() const {
    /*
     * Opportunity inputs
     */

    std::vector<double> inputs(getNbGameInputs(), 0);
    size_t i = 0;

    bool playing = m_wm->onOpportunity && (m_wm->arrival <= 2 || !CoopFixed2SharedData::fixRobotNb);
    inputs[i++] = (int) playing;
    if (fill_names) inputnames.emplace_back("playing");


    inputs[i++] = m_wm->onOpportunity;
    if (fill_names) inputnames.emplace_back("on opp");

    double nb_playing = 0;
    if (playing)
    {
        nb_playing = m_wm->nbOnOpp - 1;
        if (CoopFixed2SharedData::fixRobotNb and nb_playing > 1)
        {
            nb_playing = 1;
        }
    }
    inputs[i++] = nb_playing;
    if (fill_names) inputnames.emplace_back("nb on opp");

    if (CoopFixed2SharedData::arrivalAsInput) {
        inputs[i++] = m_wm->arrival;
        if (fill_names) inputnames.emplace_back("arrival");

    }
    if (CoopFixed2SharedData::totalInvAsInput)
    {
        inputs[i++] = m_wm->meanLastTotalInvest() / CoopFixed2SharedData::maxCoop;
        if (fill_names) inputnames.emplace_back("mean total inv");

    }
    if (CoopFixed2SharedData::ownInvAsInput) {
        inputs[i++] = m_wm->meanLastOwnInvest() / CoopFixed2SharedData::maxCoop;
        if (fill_names) inputnames.emplace_back("mean own inv");
    }

    /*
     * introspection inputs
     */
    if (CoopFixed2SharedData::selfAAsInput) {
        inputs[i++] = (m_wm->selfA  - CoopFixed2SharedData::meanA) / CoopFixed2SharedData::stdA;
        if (fill_names) inputnames.emplace_back("own A");

    }
    return inputs;
}

void CoopFixed2Controller::loadNewGenome(const std::vector<double> &newGenome)
{
    if (!CoopFixed2SharedData::splitNetwork) {
        if (m_nn->getRequiredNumberOfWeights() != newGenome.size()) {
            std::cout << m_nn->getRequiredNumberOfWeights() << "!=" << newGenome.size() << std::endl;
            exit(-1);
        }
        weights = newGenome;
        m_nn->setWeights(weights);
    } else {
        if (m_nn->getRequiredNumberOfWeights() + m_nn2->getRequiredNumberOfWeights() != newGenome.size()) {
            std::cout << m_nn->getRequiredNumberOfWeights() << "!=" << newGenome.size() << std::endl;
            exit(-1);
        }
        auto split = newGenome.begin() + m_nn->getRequiredNumberOfWeights();
        weights = std::vector<double>(newGenome.begin(), split);
        weights2 = std::vector<double>(split, newGenome.end());
        m_nn->setWeights(weights);
        m_nn2->setWeights(weights2);
    }
    if (CoopFixed2SharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman*>(m_nn)->initLastOutputs();
}

unsigned int CoopFixed2Controller::getNbInputs() const
{
    return getNbCameraInputs() + getNbGameInputs();
}

unsigned int CoopFixed2Controller::getNbGameInputs() const {
    const unsigned int nbGameInputs = static_cast<const unsigned int>(
        1 // playing
        + 1 // on Opp
        + 1 // nbOnOpportunity
        + CoopFixed2SharedData::arrivalAsInput
        + CoopFixed2SharedData::totalInvAsInput
        + CoopFixed2SharedData::ownInvAsInput
        + CoopFixed2SharedData::selfAAsInput);
    return nbGameInputs;
}

unsigned int CoopFixed2Controller::getNbCameraInputs() const {
    const unsigned int nbCameraInputs = static_cast<const unsigned int>(m_wm->_cameraSensorsNb * (5 + 2 * (int) CoopFixed2SharedData::reputation)); // dist + isWall + isRobot + isObj + nbRob + repopp + repAgent
    return nbCameraInputs;
}


double CoopFixed2Controller::getFitness() const
{
    return m_wm->_fitnessValue;
}


void CoopFixed2Controller::resetFitness()
{
    updateFitness(0);
}

void CoopFixed2Controller::updateFitness( double newFitness )
{
    if (newFitness < 0)
    {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void CoopFixed2Controller::increaseFitness( double delta )
{
    updateFitness(m_wm->_fitnessValue+delta);
}

std::string CoopFixed2Controller::inspect(std::string prefix)
{
    std::stringstream out;
    if (m_wm->fake)
    {
        out << prefix << "I'm fake robot with coop " << m_wm->fakeCoef << "\n";
    }
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
            auto coop = dynamic_cast<CoopFixed2Opportunity *>(gPhysicalObjects[entityId - gPhysicalObjectIndexStartOffset]);
            out << "with " << coop->getNbNearbyRobots() << " robots nearby.\n ";
        }
    }
    if (m_wm->onOpportunity)
    {
        out << prefix << "On opportunity with " <<  m_wm->nbOnOpp << ". I arrived " << m_wm->arrival <<".\n";
        out << prefix << "\tLast own invest: ";
        for (auto ownInvest : m_wm->lastOwnInvest)
        {
            out << ownInvest << " ";
        }
        out << "(" << m_wm->meanLastOwnInvest() << ")";
        out << "\n";
        out << prefix << "\tLast total invest: ";
        for (auto totInvest : m_wm->lastTotalInvest)
        {
            out << totInvest << " ";
        }
        out << "(" << m_wm->meanLastTotalInvest() << ")";
        out << "\n";

    }
    out << prefix << "a coeff: " << m_wm->selfA << "\n";
    out << prefix << "reputation : " << m_wm->meanLastReputation() << "\n";

    out << prefix << "Actual fitness: " << getFitness() << "\n";
    auto inputs = getInputs();
    out << prefix << "inputs:\n";
    for (int i = 0; i < inputs.size(); i++)
    {
        out << prefix << "\t" << inputnames[i] << ":" << inputs[i] << "\n";
    }
    out << prefix << "outputs:\n";
    auto& outputs = m_nn->readOut();
    for (auto& output : outputs) {
        out << prefix << "\t" << output << "\n"; // TODO Crash without reason
    }
    if (CoopFixed2SharedData::splitNetwork)
    {
        auto& outputs = m_nn2->readOut();
        for (auto& output : outputs) {
            out << prefix << "\t" << output << "\n"; // TODO Crash without reason
        }
    }
    return out.str();
}

const std::vector<double> CoopFixed2Controller::getWeights() const
{
    if (CoopFixed2SharedData::splitNetwork)
    {
        std::vector<double> allweights;
        allweights.insert(allweights.end(), weights.begin(), weights.end());
        allweights.insert(allweights.end(), weights2.begin(), weights2.end());
        return allweights;

    }
    return weights;
}

unsigned int CoopFixed2Controller::getNbOutputs() const
{
    return 2    // Motor commands
           + 1  // Cooperation value
    ;
}
