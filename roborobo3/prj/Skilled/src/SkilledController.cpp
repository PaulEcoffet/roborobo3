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
#include <Skilled/include/SkilledWorldModel.h>
#include "neuralnetworks/Elman.h"
#include "Skilled/include/SkilledController.h"
#include "Skilled/include/SkilledSharedData.h"
#include "Skilled/include/SkilledWorldObserver.h"
#include <SDL2/SDL.h>
#include <core/Utilities/Graphics.h>


enum {
    MLP_ID = 0,
    PERCEPTRON_ID = 1,
    ELMAN_ID = 2
};

SkilledController::SkilledController(RobotWorldModel *wm) : scorelogger(nullptr), m_wo(nullptr) {
    m_wm = dynamic_cast<SkilledWorldModel *>(wm);
    std::vector<unsigned int> nbNeuronsPerHiddenLayers = getNbNeuronsPerHiddenLayers();
    switch (SkilledSharedData::controllerType) {
        case MLP_ID:
            m_nn = new MLP(weights, getNbInputs(), 1,
                           nbNeuronsPerHiddenLayers, true);
            break;
        case PERCEPTRON_ID:
            m_nn = new Perceptron(weights, getNbInputs(), 1);
            break;
        case ELMAN_ID:
            m_nn = new Elman(weights, getNbInputs(), 1,
                             nbNeuronsPerHiddenLayers, true);
            break;
        default:
            std::cerr << "Invalid controller Type in " << __FILE__ << ":" << __LINE__ << ", got "
                      << SkilledSharedData::controllerType << "\n";
            exit(-1);
    }
    weights.resize(m_nn->getRequiredNumberOfWeights() + 3, 0);  // 3 for coopalone, cooppart & skill
    m_nn->setWeights(weights);
    resetFitness();
}

void SkilledController::reset() {
    m_wo = dynamic_cast<SkilledWorldObserver *>(gWorld->getWorldObserver());

    scorelogger = m_wo->getScoreLogger();
    if (SkilledSharedData::controllerType == ELMAN_ID) {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
}

void SkilledController::step() {
    verbose = 0;
    if (not m_wm->isAlive()) {
        return;
    }

    auto *best = dynamic_cast<SkilledOpportunity *>(gPhysicalObjects[0]);
    double bestscore = -9999;

    int maxopptodiscover = ceil(m_wm->getSkill() * gNbOfPhysicalObjects);
    for (size_t i = 0; i < maxopptodiscover; i++) {
        auto *opp = gPhysicalObjects[i];
        auto *lionopp = dynamic_cast<SkilledOpportunity *>(opp);
        double score = computeScoreFromOpp(lionopp, m_wo->logScore());
        if (m_wm->getId() == 0 && gVerbose) {
            //std::cout << opp->getId() << ": cost:" << cost << ", nb:" << nbopp << ", coop:" << coop << ", own:" << owncoop << ", score :" << score << std::endl;
        }
        if (score > bestscore) {
            bestscore = score;
            best = lionopp;
        }
    }


    /* Reading the output of the networks */

    m_wm->_desiredTranslationalValue = 0;
    m_wm->_desiredRotationalVelocity = 0;
    m_wm->teleport = best->getId();

    move();

    if (SkilledSharedData::asyncPlay) {
        play_and_fitness();
    }

    if (m_wm->fakeCoef < 0.8) {
        m_wm->setRobotLED_colorValues(126, 55, 49);
        if (m_wm->onOpportunity) {
            m_wm->setRobotLED_colorValues(169, 96, 89);
        }
    } else if (m_wm->fakeCoef < 1.2) {
        m_wm->setRobotLED_colorValues(0, 0, 255);
        if (m_wm->onOpportunity) {
            m_wm->setRobotLED_colorValues(100, 100, 255);
        }
    } else {
        m_wm->setRobotLED_colorValues(115, 182, 234);
        if (m_wm->onOpportunity) {
            m_wm->setRobotLED_colorValues(185, 218, 244);
        }
    }
}

double SkilledController::computeScoreFromOpp(SkilledOpportunity *testopp, bool log) {
    int onopp = testopp->isRobotOnOpp(m_wm->getId());
    int cost = (onopp) ? 0 : 1;
    int nbpart = testopp->countCurrentRobots() - onopp;
    double owncoop = getCoop(nbpart);
    double totothercoop = 0;
    if (cost && nbpart == 0) {
        return cachedEmptyOpp;
    }
    if (onopp) {
        totothercoop = testopp->getCurInv() - owncoop;
    } else {
        totothercoop = testopp->getIfNewPartInv();
    }
    double score = computeScore(cost, nbpart, owncoop, totothercoop);
    if (log) {
        const double othercoopmean = totothercoop / std::max(1, nbpart);
        scorelogger->addScore(m_wm->getId(), cost, nbpart, owncoop, othercoopmean, score);
    }
    return score;
}

std::vector<unsigned int> SkilledController::getNbNeuronsPerHiddenLayers() const {
    auto nbHiddenLayers = static_cast<unsigned int>(SkilledSharedData::nbHiddenLayers);
    std::vector<unsigned int> neuronsPerHiddenLayer(nbHiddenLayers);
    for (auto &nbNeuro : neuronsPerHiddenLayer) {
        nbNeuro = static_cast<unsigned int>(SkilledSharedData::nbNeuronsPerHiddenLayer);
    }
    return neuronsPerHiddenLayer;
}


SkilledController::~SkilledController() {
    delete m_nn;
}


void SkilledController::loadNewGenome(const std::vector<double> &newGenome) {

    auto split = newGenome.begin() + m_nn->getRequiredNumberOfWeights();
    if (m_nn->getRequiredNumberOfWeights() + 3 != newGenome.size()) {
        std::cout << "nb weights does not match nb genes: " << m_nn->getRequiredNumberOfWeights() << "!="
                  << newGenome.size() << std::endl;
        exit(-1);
    }
    weights = std::vector<double>(newGenome.begin(), split);
    weights2 = std::vector<double>(split, newGenome.end());
    assert(weights2.size() == 3);
    m_nn->setWeights(weights);
    m_wm->setCoopAlone(weights2[0] * SkilledSharedData::maxCoop);
    m_wm->setCoopPartners(weights2[1] * SkilledSharedData::maxCoop);
    m_wm->setSkill(weights2[2]);


    if (SkilledSharedData::controllerType == ELMAN_ID) {
        dynamic_cast<Elman *>(m_nn)->initLastOutputs();
    }
    cachedEmptyOpp = computeScore(1, 0, m_wm->getCoop(0), 0);
}

unsigned int SkilledController::getNbInputs() const {
    int nbInput = 3;
    if (SkilledSharedData::splitedNbPartInput) {
        nbInput += 2;
    }
    if (SkilledSharedData::costAsInput) {
        nbInput++;
    }

    return nbInput;
}


double SkilledController::getFitness() const {
    return m_wm->_fitnessValue;
}


void SkilledController::resetFitness() {
    updateFitness(0);
}

void SkilledController::updateFitness(double newFitness) {
    if (newFitness < 0) {
        updateFitness(0);
        return;
    }
    m_wm->_fitnessValue = newFitness;
}

void SkilledController::increaseFitness(double delta) {
    updateFitness(m_wm->_fitnessValue + delta);
}

std::string SkilledController::inspect(std::string prefix) {
    std::stringstream out;
    if (verbose == 0) {
        out << prefix << "I'm robot with coop coef " << m_wm->fakeCoef << "\n";

        if (m_wm->opp) {
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
    if (verbose == 1) {
        out << m_nn->toString() << std::endl;
        auto &weights = m_nn->getWeigths();
        int i = 0;
        for (auto &weight : weights) {
            out << weight << ",";
            if (i == 10) {
                out << "\n";
                i = 0;
            }
            i++;
        }
    }
    if (verbose == 2) {
        for (int i = 0; i < gInitialNumberOfRobots; i++) {
            out << prefix << i << ": " << m_wm->getCoop(i) << "(" << m_wm->getCoop(i, true) << ")\n";
        }
    }
    verbose = (verbose + 1) % 3;
    return out.str();
}

std::vector<double> SkilledController::getWeights() const {
    std::vector<double> allweights;

    allweights.insert(allweights.end(), weights.begin(), weights.end());
    allweights.insert(allweights.end(), weights2.begin(), weights2.end());

    return allweights;
}

unsigned int SkilledController::getNbOutputs() const {
    return 1;
}

void SkilledController::move() {
    int dest_obj = m_wm->teleport;
    double angle = ((double) m_wm->getId() / gNbOfRobots) * 2 * M_PI;
    if (dest_obj != -1) {
        PhysicalObject *physobj = gPhysicalObjects[dest_obj];
        auto rob = gWorld->getRobot(this->m_wm->getId());
        rob->unregisterRobot();
        rob->setCoord(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + ((double) gRobotWidth / 2))),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + (double) gRobotWidth / 2)));
        rob->setCoordReal(
                static_cast<int>(physobj->getXCenterPixel() +
                                 cos(angle) * (gPhysicalObjectDefaultRadius + (double) gRobotWidth / 2)),
                static_cast<int>(physobj->getYCenterPixel() +
                                 sin(angle) * (gPhysicalObjectDefaultRadius + (double) gRobotWidth / 2)));
        rob->getWorldModel()->_agentAbsoluteOrientation = 0;
        rob->registerRobot();
    }


    Uint8 r, g, b;
    Uint32 pixel = getPixel32(gFootprintImage,
                              static_cast<int>(m_wm->_xReal + 0.5),  // NOLINT(bugprone-incorrect-roundings)
                              static_cast<int>(m_wm->_yReal + 0.5));  // NOLINT(bugprone-incorrect-roundings)
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


        if (!m_wm->opp || targetIndex != m_wm->opp->getId()) {
            newopp = true;
            if (m_wm->opp) {
                m_wm->opp->removeRobot(m_wm->getId());
            }
        }
        m_wm->opp = dynamic_cast<SkilledOpportunity *>(gPhysicalObjects[targetIndex]); // Agent is on this opp

        if (m_wm->teleport && dest_obj != -1 && targetIndex != dest_obj) {
            std::cerr << "Not on opp for tp : " << m_wm->getId() << " :" << targetIndex << " " << dest_obj << "\n";
            m_wm->setRobotLED_colorValues(0, 0, 0);
            //exit(1);
        }
    }
    m_wm->newopp = newopp;
}

void SkilledController::play_and_fitness() {
    if (!m_wm->isAlive() or m_wm->opp == nullptr) {
        return;
    }
    double cost = (m_wm->newopp) ? SkilledSharedData::cost : 0;

    auto totalinv = m_wm->opp->getCurInv();
    int n = m_wm->opp->countCurrentRobots();
    assert(n >= 1);
    double payoff = SkilledWorldObserver::payoff(m_wm->getCoop(n - 1), totalinv, n, SkilledSharedData::meanA,
                                                 SkilledSharedData::b);
    m_wm->_fitnessValue += (payoff - cost) / SkilledSharedData::evaluationTime; // Normalized with time
    dynamic_cast<SkilledWorldObserver *>(gWorld->getWorldObserver())->logAgent(m_wm);

}

double SkilledController::getCoop(int i) {
    double coop = m_wm->getCoop(i);
    return coop;
}

double SkilledController::getCoopWeight() {
    return weights2[0];
}


static inline double normalize(const double value, const double min = 0, const double max = 1) {
    return ((value - min) / (max - min)) * 2 - 1;
}

double SkilledController::computeScore(int cost, int nbPart, double owncoop, double totothercoop) {
    assert(cost == 0 || cost == 1);
    assert(nbPart >= 0 && nbPart <= gInitialNumberOfRobots);
    assert(owncoop >= 0 && owncoop <= SkilledSharedData::maxCoop * (1 + SkilledSharedData::fakeCoef + 0.01));
    assert(totothercoop >= 0 &&
           totothercoop <= nbPart * SkilledSharedData::maxCoop * (1 + SkilledSharedData::fakeCoef + 0.01));

    std::vector<double> inputs(getNbInputs(), 0);
    int i = 0;
    if (SkilledSharedData::costAsInput) {
        inputs[i++] = cost;
    }
    if (SkilledSharedData::splitedNbPartInput) {
        int curnbpart = nbPart;
        for (int nbdec = 0; nbdec < 3; nbdec++) {
            inputs[i++] = (double) (nbPart % 10) / 10.0;  // split units, tens and hundreds
            curnbpart = curnbpart / 10;
        }
    } else {
        inputs[i++] = (double) nbPart / gNbOfRobots;
    }
    if (SkilledSharedData::otherInvAsInput) {
        inputs[i++] = totothercoop / (SkilledSharedData::maxCoop * std::max(nbPart, 1));
    } else {
        inputs[i++] = 0;
    }

    inputs[i] = owncoop / SkilledSharedData::maxCoop;

    double score = 0;

    if (SkilledSharedData::optimalPlay) {
        score = SkilledWorldObserver::payoff(owncoop, totothercoop + owncoop, (nbPart + 1), SkilledSharedData::meanA,
                                             SkilledSharedData::b)
                - cost * SkilledSharedData::cost;
        assert(score < 1000000 && score > -1000000); // ensure score is never infinite
    } else {
        m_nn->setInputs(inputs);
        m_nn->step();
        score = m_nn->readOut()[0];
        assert(score >= -1 && score <= 1);
    }

    return score;
}