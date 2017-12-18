//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_FASTWANDERERWORLDOBSERVER_H
#define ROBOROBO3_FASTWANDERERWORLDOBSERVER_H


#include <core/Observers/WorldObserver.h>
#include <core/World/World.h>
#include <contrib/network/NetworkInterface.h>
#include "core/Utilities/LogManager.h"
#include "contrib/json/json.hpp"
#include "FastWandererController.h"
#include "contrib/network/PyevoInterface.h"

using json = nlohmann::json;

class FastWandererWorldObserver : public WorldObserver
{
public:
    explicit FastWandererWorldObserver(World *__world);
    ~FastWandererWorldObserver() override;

    void stepPre() override;
    void stepPost() override;
    void reset() override;

    std::vector<std::pair<int, double>> getSortedFitnesses() const;

    void logFitnesses(const std::vector<std::pair<int, double>>& sortedFitnesses);
    void resetEnvironment();

protected:
    LogManager *m_fitnessLogManager;

    int m_curEvalutionIteration;
    int _generationCount;
    PyevoInterface pycma;

    void clearRobotFitnesses();


    void loadNextGeneration(const std::vector<std::vector<double>> &generation);
};


#endif //ROBOROBO3_FASTWANDERERWORLDOBSERVER_H
