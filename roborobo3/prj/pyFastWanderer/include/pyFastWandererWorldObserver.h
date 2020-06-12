//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_PYFASTWANDERERWORLDOBSERVER_H
#define ROBOROBO3_PYFASTWANDERERWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/NetworkInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "pyFastWandererController.h"
#include "network/PyevoInterface.h"

using json = nlohmann::json;

class pyFastWandererWorldObserver : public WorldObserver
{
public:
    explicit pyFastWandererWorldObserver(World *__world);
    ~pyFastWandererWorldObserver() override;

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


#endif //ROBOROBO3_PYFASTWANDERERWORLDOBSERVER_H
