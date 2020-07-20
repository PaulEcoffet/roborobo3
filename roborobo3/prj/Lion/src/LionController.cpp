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
#include "Lion/include/LionWorldObserver.h"
#include <SDL2/SDL.h>
#include <core/Utilities/Graphics.h>


enum
{
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

LionController::LionController(RobotWorldModel *wm) : scorelogger(nullptr)
{
    m_wm = dynamic_cast<LionWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    std::vector<unsigned int> nbNeurons2 = std::vector<unsigned int>(1, 3);
    switch (LionSharedData::controllerType)
    {
        case MLP_ID:
            m_nn = new MLP(weights, getNbInputs(), 1,
                           nbNeuronsPerHiddenLayers, true);
            m_nn2 = new MLP(weights, 2, 1, nbNeurons2, true);
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(weights, getNbInputs(), 1);
            m_nn2 = new Perceptron(weights, 1, 1);
            break;
        case ELMAN_ID:
            m_nn = new Elman(weights, getNbInputs(), 1,
                             nbNeuronsPerHiddenLayers, true);
            m_nn2 = new Elman(weights, 1, 1,
                              nbNeurons2, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << LionSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights(), 0);
    m_nn->setWeights(weights);
    if (LionSharedData::independantCoop == 0)
    {
        weights2.resize(m_nn2->getRequiredNumberOfWeights(), 0);
        m_nn2->setWeights(weights2);
    }
    else
    {
        weights2.clear();
    }
    resetFitness();
}

void LionController::reset()
{
    m_wo = dynamic_cast<LionWorldObserver *>(gWorld->getWorldObserver());

    scorelogger = m_wo->getScoreLogger();
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

    auto *best = dynamic_cast<LionOpportunity *>(gPhysicalObjects[0]);
    double bestscore = -9999;


    for (auto *opp : gPhysicalObjects)
    {
        auto *lionopp = dynamic_cast<LionOpportunity *>(opp);
        double score = computeScoreFromOpp(lionopp, m_wm->opp, m_wo->logScore());
        if (m_wm->getId() == 0 && gVerbose)
        {
            //std::cout << opp->getId() << ": cost:" << cost << ", nb:" << nbopp << ", coop:" << coop << ", own:" << owncoop << ", score :" << score << std::endl;
        }
        if (score > bestscore)
        {
            bestscore = score;
            best = lionopp;
        }
    }


    /* Reading the output of the networks */

    m_wm->_desiredTranslationalValue = 0;
    m_wm->_desiredRotationalVelocity = 0;
    m_wm->teleport = best->getId();

    move();

    if (LionSharedData::asyncPlay)
    {
        play_and_fitness();
    }

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

double LionController::computeScoreFromOpp(LionOpportunity *testopp, LionOpportunity *curopp, bool log)
{
    int onopp = testopp->isRobotOnOpp(m_wm->getId());
    int cost = (onopp) ? 0 : 1;
    int nbpart = testopp->countCurrentRobots() - onopp;
    double owncoop = getCoop(nbpart);
    double totothercoop = 0;
    if (cost && nbpart == 0)
    {
        return cachedEmptyOpp;
    }
    if (onopp)
    {
        totothercoop = testopp->getCurInv() - owncoop;
    }
    else
    {
        totothercoop = testopp->getIfNewPartInv();
    }
    double score = computeScore(cost, nbpart, owncoop, totothercoop);
    if (log)
    {
        const double othercoopmean = totothercoop / std::max(1, nbpart);
        scorelogger->addScore(m_wm->getId(), cost, nbpart, owncoop, othercoopmean, score);
    }
    return score;
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
    if (m_nn->getRequiredNumberOfWeights() + weights2.size() != newGenome.size())
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

    std::vector<double> inputs(2, 0);
    if (LionSharedData::hardCoop)
    {
        assert(LionSharedData::independantCoop == 0);
        for (int i = 0; i < gInitialNumberOfRobots; i++)
        {
            int nbPlayers = i + 1;
            double coop = 0;
            if (getCoopWeight() < 0)
            {
                coop = LionSharedData::meanA / nbPlayers;
            }
            else
            {
                coop = LionSharedData::meanA + (LionSharedData::b * (nbPlayers - 1)) / nbPlayers;
            }
            m_wm->setCoop(i, coop);
        }
    }
    else if (LionSharedData::independantCoop == 1)
    {
        m_wm->setCoop(0, LionSharedData::meanA);
        if (m_wm->fakeCoef < 1)
        {
            m_wm->setCoop(1, LionSharedData::meanA / 2);
        }
        else
        {
            m_wm->setCoop(1, LionSharedData::meanA + LionSharedData::b / 2);
        }
    }
    else if (LionSharedData::independantCoop == 2)
    {
        const double coefrange = 2 * LionSharedData::fakeCoef;
        const double ess = LionSharedData::meanA / 2;
        const double so = LionSharedData::meanA + LionSharedData::b / 2;
        double coef = (m_wm->fakeCoef - (1 - LionSharedData::fakeCoef)) / coefrange;
        m_wm->setCoop(0, LionSharedData::meanA);
        m_wm->setCoop(1, ess * (1 - coef) + so * coef);
    }
    else
    {
        for (int i = 0; i < gInitialNumberOfRobots; i++)
        {
            inputs[0] = (double) (i % 10) / 10;  // Units counter
            inputs[1] = (double) (i / 10) /
                        ((double) gInitialNumberOfRobots / 10); // ten counter  -- NOLINT(bugprone-integer-division)
            m_nn2->setInputs(inputs);
            m_nn2->step();
            auto &output = m_nn2->readOut();
            double coop = ((output[0] + 1) / 2) * LionSharedData::maxCoop;
            m_wm->setCoop(i, coop);
        }
    }
    cachedEmptyOpp = computeScore(1, 0, m_wm->getCoop(0), 0);
}

unsigned int LionController::getNbInputs() const
{
    int nbInput = 3;
    if (LionSharedData::splitedNbPartInput)
    {
        nbInput += 2;
    }
    if (LionSharedData::costAsInput)
    {
        nbInput++;
    }

    return nbInput;
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
            out << m_wm->getCoop(m_wm->opp->countCurrentRobots() - 1) << " ("
                << m_wm->getCoop(m_wm->opp->countCurrentRobots() - 1, true) << ")";
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
        auto &weights = m_nn->getWeigths();
        int i = 0;
        for (auto &weight : weights)
        {
            out << weight << ",";
            if (i == 10)
            {
                out << "\n";
                i = 0;
            }
            i++;
        }
    }
    if (verbose == 2)
    {
        for (int i = 0; i < gInitialNumberOfRobots; i++)
        {
            out << prefix << i << ": " << m_wm->getCoop(i) << "(" << m_wm->getCoop(i, true) << ")\n";
        }
    }
    verbose = (verbose + 1) % 3;
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

void LionController::move()
{
    int dest_obj = m_wm->teleport;
    double angle = ((double) m_wm->getId() / gNbOfRobots) * 2 * M_PI;
    if (dest_obj != -1)
    {
        PhysicalObject *physobj = gPhysicalObjects[dest_obj];
        auto rob = gWorld->getRobot(this->m_wm->getId());
        rob->unregisterRobot();
        rob->setCoord(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
        rob->setCoordReal(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + gRobotWidth / 2)));
        rob->getWorldModel()->_agentAbsoluteOrientation = 0;
        rob->registerRobot();
    }


    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage, static_cast<int>(m_wm->_xReal + 0.5),
                              static_cast<int>(m_wm->_yReal + 0.5));
    SDL_GetRGB(pixel, gFootprintImage->format, &r, &g, &b);
    m_wm->_groundSensorValue[0] = (int) r;
    m_wm->_groundSensorValue[1] = (int) g;
    m_wm->_groundSensorValue[2] = (int) b;


    int targetIndex = m_wm->getGroundSensorValue();
    bool newopp = false;

    if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset +
                                                                        (int) gPhysicalObjects.size())   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        assert(targetIndex == dest_obj);

        gPhysicalObjects[targetIndex]->isWalked(m_wm->getId() + gRobotIndexStartOffset); // callback on opportunity


        if (!m_wm->opp || targetIndex != m_wm->opp->getId())
        {
            newopp = true;
            if (m_wm->opp)
            {
                if (gVerbose && false)
                {
                    std::cout << m_wm->getId() << " moved from " << m_wm->opp->getId() << " to " << targetIndex
                              << std::endl;
                }
                m_wm->opp->removeRobot(m_wm->getId());
            }
        }
        m_wm->opp = dynamic_cast<LionOpportunity *>(gPhysicalObjects[targetIndex]); // Agent is on this opp

        if (m_wm->teleport && dest_obj != -1 && targetIndex != dest_obj)
        {
            std::cerr << "Not on opp for tp : " << m_wm->getId() << " :" << targetIndex << " " << dest_obj << "\n";
            m_wm->setRobotLED_colorValues(0, 0, 0);
            //exit(1);
        }
    }
    m_wm->newopp = newopp;
}

void LionController::play_and_fitness()
{
    if (!m_wm->isAlive() or m_wm->opp == nullptr)
    {
        return;
    }
    double cost = (m_wm->newopp) ? LionSharedData::cost : 0;

    auto totalinv = m_wm->opp->getCurInv();
    int n = m_wm->opp->countCurrentRobots();
    assert(n >= 1);
    double payoff = LionWorldObserver::payoff(m_wm->getCoop(n - 1), totalinv, n, LionSharedData::meanA,
                                              LionSharedData::b);
    m_wm->_fitnessValue += (payoff - cost) / LionSharedData::evaluationTime; // Normalized with time
    dynamic_cast<LionWorldObserver *>(gWorld->getWorldObserver())->logAgent(m_wm);

}

double LionController::getCoop(int i)
{
    double coop = m_wm->getCoop(i);
    return coop;
}

double LionController::getCoopWeight()
{
    return weights2[0];
}


static inline double normalize(const double value, const double min = 0, const double max = 1)
{
    return ((value - min) / (max - min)) * 2 - 1;
}

double LionController::computeScore(int cost, int nbPart, double owncoop, double totothercoop)
{
    assert(cost == 0 || cost == 1);
    assert(nbPart >= 0 && nbPart <= gInitialNumberOfRobots);
    assert(owncoop >= 0 && owncoop <= LionSharedData::maxCoop * (1 + LionSharedData::fakeCoef + 0.01));
    assert(totothercoop >= 0 &&
           totothercoop <= nbPart * LionSharedData::maxCoop * (1 + LionSharedData::fakeCoef + 0.01));

    std::vector<double> inputs(getNbInputs(), 0);
    int i = 0;
    if (LionSharedData::costAsInput)
    {
        inputs[i++] = cost;
    }
    if (LionSharedData::splitedNbPartInput)
    {
        int curnbpart = nbPart;
        for (int nbdec = 0; nbdec < 3; nbdec++)
        {
            inputs[i++] = (double) (nbPart % 10) / 10.0;  // split units, tens and hundreds
            curnbpart = curnbpart / 10;
        }
    }
    else
    {
        inputs[i++] = (double) nbPart / gNbOfRobots;
    }
    if (LionSharedData::otherInvAsInput)
    {
        inputs[i++] = totothercoop / (LionSharedData::maxCoop * std::max(nbPart, 1));
    }
    else
    {
        inputs[i++] = 0;
    }

    inputs[i] = owncoop / LionSharedData::maxCoop;

    double score = 0;

    if (LionSharedData::optimalPlay)
    {
        score = LionWorldObserver::payoff(owncoop, totothercoop + owncoop, (nbPart + 1), LionSharedData::meanA,
                                          LionSharedData::b)
                - cost * LionSharedData::cost;
        assert(score < 1000000 && score > -1000000); // ensure score is never infinite
    }
    else
    {
        m_nn->setInputs(inputs);
        m_nn->step();
        score = m_nn->readOut()[0];
        assert(score >= -1 && score <= 1);
    }

    return score;
}