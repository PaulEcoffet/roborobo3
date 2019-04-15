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

std::vector<std::string> LionController::inputnames;

LionController::LionController(RobotWorldModel *wm)
{
    m_wm = dynamic_cast<LionWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    unsigned int nbCamInputs = getNbCameraInputs();
    unsigned int nbGameInputs = getNbGameInputs();

    unsigned int nbMoveOutput = 1 + (int) !LionSharedData::tpToNewObj;
    unsigned int nbGameOutput = (int) !LionSharedData::fixCoop + (int) LionSharedData::punishment;

    fill_names = true;
    fillNames();
    switch (LionSharedData::controllerType)
    {
        case MLP_ID:
            if (LionSharedData::splitNetwork && !LionSharedData::onlyNforGame)
            {
                std::vector<unsigned int> nbNeurons2 = {2}; // TODO 2 should not be hardcode
                m_nn = new MLP(weights, nbCamInputs + nbGameInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new MLP(weights2, nbGameInputs, nbGameOutput, nbNeurons2, true, false, 1.0);
            }
            else if (LionSharedData::onlyNforGame)
            {
                assert(LionSharedData::splitNetwork);
                assert(!LionSharedData::fixCoop);
                std::vector<unsigned int> nbNeurons2 = {2}; // best value for the fit
                m_nn = new MLP(weights, nbCamInputs + nbGameInputs, nbMoveOutput,
                               nbNeuronsPerHiddenLayers, true);
                m_nn2 = new MLP(weights2, 1, 1, nbNeurons2, true, false, 1.0);
            }
            else
            {
                m_nn = new MLP(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput,
                               nbNeuronsPerHiddenLayers, true);
            }
            break;
        case PERCEPTRON_ID:
            if (LionSharedData::splitNetwork)
            {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput);
                m_nn2 = new Perceptron(weights2, nbGameOutput, nbGameOutput);
            }
            else if (LionSharedData::onlyNforGame)
            {
                throw std::string("Not implemented");
            }
            else
            {
                m_nn = new Perceptron(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput);
            }
            break;
        case ELMAN_ID:
            if (LionSharedData::splitNetwork)
            {
                m_nn = new Elman(weights, nbCamInputs + nbGameInputs, nbMoveOutput, nbNeuronsPerHiddenLayers, true);
                m_nn2 = new Elman(weights2, nbGameOutput, nbGameOutput, 2, true);
            }
            else if (LionSharedData::onlyNforGame)
            {
                throw std::string("Not implemented");
            }
            else
            {
                m_nn = new Elman(weights, nbCamInputs + nbGameInputs, nbMoveOutput + nbGameOutput,
                                 nbNeuronsPerHiddenLayers, true);
            }
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << LionSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);

    if (LionSharedData::splitNetwork)
    {
        weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
        m_nn2->setWeights(weights);
    }

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

    int curoppid = (m_wm->opp) ? m_wm->getId() : -1;
    for (auto *opp : gPhysicalObjects)
    {
        auto *lionopp = dynamic_cast<LionOpportunity *>(opp);
        double cost = opp->getId() == curoppid;

        std::vector<double> inputs = getInputs();
        m_nn->setInputs(inputs);
        m_nn->step();
        std::vector<double> outputs = m_nn->readOut();
        if (outputs[0] > bestscore)
        {
            bestscore = outputs[0];
            best = opp;
        }
    }


    /* Reading the output of the networks */

    m_wm->_desiredTranslationalValue = 0;
    m_wm->_desiredRotationalVelocity = 0;
    m_wm->teleport = best_spot;


    m_wm->_cooperationLevel = hardcoop; // Range between [0; maxCoop]


    if (m_wm->fakeCoef < 0.8)
    {
        m_wm->setRobotLED_colorValues(126, 55, 49);
        if (m_wm->onOpportunity && (!LionSharedData::fixRobotNb || m_wm->arrival <= 2))
        {
            m_wm->setRobotLED_colorValues(169, 96, 89);
        }
    }
    else if (m_wm->fakeCoef < 1.2)
    {
        m_wm->setRobotLED_colorValues(0, 0, 255);
        if (m_wm->onOpportunity && (!LionSharedData::fixRobotNb || m_wm->arrival <= 2))
        {
            m_wm->setRobotLED_colorValues(100, 100, 255);
        }
    }
    else
    {
        m_wm->setRobotLED_colorValues(115, 182, 234);
        if (m_wm->onOpportunity && (!LionSharedData::fixRobotNb || m_wm->arrival <= 2))
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

std::vector<double> LionController::getInputs()
{
    size_t i;
    std::vector<double> inputs;
    inputs = getCameraInputs();

    const std::vector<double> game_inputs(getGameInputs());
    inputs.insert(inputs.end(), game_inputs.begin(), game_inputs.end());
    fill_names = false;

    assert((LionSharedData::splitNetwork && inputs.size() == m_nn->getNbInputs() + m_nn2->getNbInputs()) ||
           inputs.size() == m_nn->getNbInputs());
    assert(inputnames.size() == inputs.size());
    return inputs;
}

std::vector<double> LionController::getCameraInputs() const
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
            auto *opportunity = dynamic_cast<LionOpportunity *>(
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
        if (LionSharedData::reputation)
        {
            inputs[i++] = reputation / LionSharedData::maxCoop;
            inputs[i++] = nbplays;
        }
        inputs[i++] = (entityId == WALL_ID) ? dist : 1;
        inputs[i++] = (isOpportunity) ? dist : 1;
        inputs[i++] = nbOnOpp;

        if (LionSharedData::reputation)
        {
            inputs[i++] = lastInvOnOpp / LionSharedData::maxCoop;
        }

    }

    return inputs;
}


void LionController::fillNames()
{
    if (inputnames.empty())
    {
        for (int j = 0; j < m_wm->_cameraSensorsNb; j++)
        {
            //inputnames.emplace_back("dist " + std::to_string(j));
            inputnames.emplace_back("dist robot");
            if (LionSharedData::reputation)
            {
                inputnames.emplace_back("reputation");
                inputnames.emplace_back("nb plays");
            }
            inputnames.emplace_back("dist wall");
            inputnames.emplace_back("dist obj");
            inputnames.emplace_back("nb on obj");
            if (LionSharedData::reputation)
            {
                inputnames.emplace_back("last inv on opp");
            }
        }

        inputnames.emplace_back("playing");
        inputnames.emplace_back("on opp");
        inputnames.emplace_back("nb on opp");
        if (LionSharedData::arrivalAsInput)
        {
            inputnames.emplace_back("arrival");
        }
        if (LionSharedData::totalInvAsInput)
        {
            inputnames.emplace_back("mean total inv");

        }
        if (LionSharedData::ownInvAsInput)
        {
            inputnames.emplace_back("mean own inv");
        }

        if (LionSharedData::punishmentAsInput)
        {
            inputnames.emplace_back("punishment");
        }

        /*
         * introspection inputs
         */
        if (LionSharedData::selfAAsInput)
        {
            inputnames.emplace_back("own A");
        }
        fill_names = false;
    }
}

std::vector<double> LionController::getGameInputs() const
{
    /*
     * Opportunity inputs
     */

    std::vector<double> inputs(getNbGameInputs(), 0);
    size_t i = 0;
    bool playing = m_wm->isPlaying();
    inputs[i++] = (int) playing;
    inputs[i++] = m_wm->onOpportunity;

    double nb_playing = 0;
    if (playing)
    {
        nb_playing = m_wm->nbOnOpp - 1;
        if (LionSharedData::fixRobotNb and nb_playing > 1)
        {
            nb_playing = 1;
        }
    }
    inputs[i++] = nb_playing;

    if (LionSharedData::arrivalAsInput)
    {
        inputs[i++] = m_wm->arrival;
    }
    if (LionSharedData::totalInvAsInput)
    {
        inputs[i++] = m_wm->meanLastTotalInvest() / LionSharedData::maxCoop;
    }
    if (LionSharedData::ownInvAsInput)
    {
        inputs[i++] = m_wm->meanLastOwnInvest() / LionSharedData::maxCoop;
    }

    if (LionSharedData::punishmentAsInput)
    {
        inputs[i++] = m_wm->punishment / LionSharedData::maxCoop;
    }

    /*
     * introspection inputs
     */
    if (LionSharedData::selfAAsInput)
    {
        inputs[i++] = (m_wm->selfA - LionSharedData::meanA) / LionSharedData::stdA;
    }
    return inputs;
}

void LionController::loadNewGenome(const std::vector<double> &newGenome)
{
    int coopgene = 0;
    if (LionSharedData::fixCoop)
    {
        coopgene = 1;
    }
    if (!LionSharedData::splitNetwork)
    {
        if (m_nn->getRequiredNumberOfWeights() + coopgene != newGenome.size())
        {
            std::cout << "nb weights does not match nb genes: " << m_nn->getRequiredNumberOfWeights() << "!="
                      << newGenome.size() << std::endl;
            exit(-1);
        }
        weights = std::vector<double>(newGenome.begin() + coopgene, newGenome.end());
        m_nn->setWeights(weights);
    }
    else
    {
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
    }
    if (LionSharedData::fixCoop)
    {
        hardcoop = newGenome[0] * LionSharedData::maxCoop;
    }
    if (LionSharedData::controllerType == ELMAN_ID)
    {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

unsigned int LionController::getNbInputs() const
{
    return getNbCameraInputs() + getNbGameInputs();
}

unsigned int LionController::getNbGameInputs() const
{
    const auto nbGameInputs = static_cast<const unsigned int>(
            1 // playing
            + 1 // on Opp
            + 1 // nbOnOpportunity
            + LionSharedData::arrivalAsInput
            + LionSharedData::totalInvAsInput
            + LionSharedData::ownInvAsInput
            + LionSharedData::selfAAsInput
            + LionSharedData::punishmentAsInput
    );
    return nbGameInputs;
}

unsigned int LionController::getNbCameraInputs() const
{
    const unsigned int nbCameraInputs = static_cast<const unsigned int>(
            m_wm->_cameraSensorsNb * (4 + 3 *
                                          (int) LionSharedData::reputation)); // dist + isWall + isRobot + isObj + nbRob + repopp + repAgent + nbplays
    return nbCameraInputs;
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
                auto coop = dynamic_cast<LionOpportunity *>(gPhysicalObjects[entityId -
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
        out << prefix << "last coop: " << m_wm->_cooperationLevel << "\n";
        out << prefix << "reputation : " << m_wm->meanLastCommonKnowledgeReputation() << "\n";
        out << prefix << "received punishment : " << m_wm->punishment << "\n";
        out << prefix << "sent punishment : " << m_wm->spite << "\n";

        out << prefix << "Actual fitness: " << getFitness() << "\n";
    }
    if (verbose == 1)
    {
        auto inputs = getInputs();
        out << prefix << "inputs:\n";
        for (int i = 0; i < inputs.size(); i++)
        {
            out << prefix << "\t" << inputnames[i] << ":" << inputs[i] << "\n";
        }
        out << prefix << "outputs:\n";
        auto &outputs = m_nn->readOut();
        for (auto &output : outputs)
        {
            out << prefix << "\t" << output << "\n"; // TODO Crash without reason
        }
        if (LionSharedData::splitNetwork)
        {
            auto &outputs = m_nn2->readOut();
            for (auto &output : outputs)
            {
                out << prefix << "\t" << output << "\n"; // TODO Crash without reason
            }
        }
    }
    if (verbose == 2)
    {
        out << m_nn->toString() << std::endl;
        if (LionSharedData::splitNetwork)
        {
            out << m_nn2->toString() << std::endl;
        }
    }
    verbose++;
    return out.str();
}

const std::vector<double> LionController::getWeights() const
{
    std::vector<double> allweights;
    if (LionSharedData::fixCoop)
    {
        allweights.push_back(hardcoop);
    }
    allweights.insert(allweights.end(), weights.begin(), weights.end());

    if (LionSharedData::splitNetwork)
    {
        allweights.insert(allweights.end(), weights2.begin(), weights2.end());
    }
    return allweights;
}

unsigned int LionController::getNbOutputs() const
{
    return 1
           + (unsigned int) !LionSharedData::tpToNewObj // Motor commands
           + (unsigned int) !LionSharedData::fixCoop  // Cooperation value
           + (unsigned int) LionSharedData::punishment;
}
