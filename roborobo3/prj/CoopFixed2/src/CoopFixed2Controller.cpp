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

CoopFixed2Controller::CoopFixed2Controller(RobotWorldModel* wm)
{
    m_wm = dynamic_cast<CoopFixed2WorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbInputs = getNbInputs();
    unsigned int nbOutputs = getNbOutputs();

    switch (CoopFixed2SharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(weights, nbInputs, nbOutputs);
            break;
        case ELMAN_ID:
            m_nn = new Elman(weights, nbInputs, nbOutputs, nbNeuronsPerHiddenLayers, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "<< CoopFixed2SharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);


    m_nn->setWeights(weights);
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

    std::vector<double> inputs = getInputs();
    /*
    std::cout << "Input: \n";
    for (const auto input : inputs) {
        std::cout << input << "\n";
    }
     */
    m_nn->setInputs(inputs);
    m_nn->step();

    std::vector<double> outputs = m_nn->readOut();

    for (const auto output : outputs) {
        if (isnan(output)) {
            for (int i = 0; i < inputs.size(); i++)
            {
                std::cerr << "\t" << inputnames[i] << ":" << inputs[i] << "\n";
            }
            std::cerr << "NaN in output.\nExiting\n";
            exit(-1);
        }
    }

    if (CoopFixed2SharedData::tpToNewObj) {
        m_wm->_desiredTranslationalValue = 1;
        m_wm->_desiredRotationalVelocity = 0;
        m_wm->teleport = outputs[0] > 0;

    } else {
        m_wm->_desiredTranslationalValue = outputs[0] * gMaxTranslationalSpeed;
        m_wm->_desiredRotationalVelocity = outputs[1] * gMaxRotationalSpeed;

    }
    m_wm->_cooperationLevel = ((outputs[2] + 1) / 2) * CoopFixed2SharedData::maxCoop; // Range between [0; maxCoop]

    if (m_wm->fake)
    {
        if (m_wm->fakeCoef < 1)
            m_wm->setRobotLED_colorValues(255, 0, 0);
        else
            m_wm->setRobotLED_colorValues(127, 255, 255);
    }
    else
    {
        if (m_wm->onOpportunity)
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
    const int WALL_ID = 0;
    bool fill_names = inputnames.empty();
    size_t i = 0;
    std::vector<double> inputs(m_nn->getNbInputs());

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
            inputnames.emplace_back("dist " + std::to_string(j));
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

    /*
     * Opportunity inputs
     */
    inputs[i++] = m_wm->onOpportunity;
    if (fill_names) inputnames.emplace_back("on opp");

    inputs[i++] = m_wm->nbOnOpp;
    if (fill_names) inputnames.emplace_back("nb on opp");

    if (CoopFixed2SharedData::arrivalAsInput) {
        inputs[i++] = m_wm->arrival;
        if (fill_names) inputnames.emplace_back("arrival");

    }
    if (CoopFixed2SharedData::totalInvAsInput)
    {
        inputs[i++] = m_wm->meanLastTotalInvest();
        if (fill_names) inputnames.emplace_back("mean total inv");

    }
    if (CoopFixed2SharedData::ownInvAsInput) {
        inputs[i++] = m_wm->meanLastOwnInvest();
        if (fill_names) inputnames.emplace_back("mean own inv");
    }

    /*
     * introspection inputs
     */
    if (CoopFixed2SharedData::selfAAsInput) {
        inputs[i++] = m_wm->selfA;
        if (fill_names) inputnames.emplace_back("own A");

    }

    assert(inputs.size() == m_nn->getNbInputs());
    assert(inputnames.size() == inputs.size());
    return inputs;
}

void CoopFixed2Controller::loadNewGenome(const std::vector<double> &newGenome)
{
    if(m_nn->getRequiredNumberOfWeights() != newGenome.size())
    {
        std::cout << m_nn->getRequiredNumberOfWeights() << "!=" << newGenome.size() << std::endl;
        exit(-1);
    }
    weights = newGenome;
    m_nn->setWeights(weights);
    if (CoopFixed2SharedData::controllerType == ELMAN_ID)
        dynamic_cast<Elman*>(m_nn)->initLastOutputs();
}

unsigned int CoopFixed2Controller::getNbInputs() const
{

    return static_cast<unsigned int>(
            m_wm->_cameraSensorsNb * (5 + 2 * (int) CoopFixed2SharedData::reputation) // dist + isWall + isRobot + isObj + nbRob + repopp + repAgent
            + 1 // on Opp
            + 1 // nbOnOpportunity
            + CoopFixed2SharedData::arrivalAsInput
            + CoopFixed2SharedData::totalInvAsInput
            + CoopFixed2SharedData::ownInvAsInput
            + CoopFixed2SharedData::selfAAsInput
    );
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
    return out.str();
}

const std::vector<double>& CoopFixed2Controller::getWeights() const
{
    return weights;
}

unsigned int CoopFixed2Controller::getNbOutputs() const
{
    return 2    // Motor commands
           + 1  // Cooperation value
    ;
}
