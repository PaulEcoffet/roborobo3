//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_MAXONEWORLDOBSERVER_H
#define ROBOROBO3_MAXONEWORLDOBSERVER_H



#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include <queue>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "MaxOneController.h"
/*
#include <opencv2/core.hpp>  // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // VideoWriter
#include <opencv2/imgproc/imgproc.hpp>  // channel manipulation
*/
using json = nlohmann::json;

class MaxOneWorldObserver : public WorldObserver
{
public:
    explicit MaxOneWorldObserver(World *__world);

    ~MaxOneWorldObserver() override;

    void reset() override;

    void resetEnvironment();

    void stepPre() override;

    void stepPost() override;

    void stepEvolution();


protected:
    World *m_world;

    int m_generationCount = 0;
    int m_curEvaluationIteration = 0;
    int m_curEvaluationInGeneration = 0;
    int m_nbIndividuals = 0;
    std::vector<std::vector<double>> m_individuals;
    PyevoInterface pyevo;


    void loadGenomesInRobots(const std::vector<std::vector<double>> &genomes);

};


#endif //ROBOROBO3_MAXONEWORLDOBSERVER_H
