//
// Created by paul on 30/10/17.
//

#ifndef ROBOROBO3_NegociateGymTrainWorldObserver_H
#define ROBOROBO3_NegociateGymTrainWorldObserver_H


#include <Observers/WorldObserver.h>
#include <World/World.h>
#include <network/PyevoInterface.h>
#include "Utilities/LogManager.h"
#include "json/json.hpp"
#include "NegociateGymController.h"
#include "NegociateGymWorldObserver.h"
/*
#include <opencv2/core.hpp>  // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // VideoWriter
#include <opencv2/imgproc/imgproc.hpp>  // channel manipulation
*/
using json = nlohmann::json;

class NegociateGymTrainWorldObserver : public NegociateGymWorldObserver
{
public:
    using NegociateGymWorldObserver::NegociateGymWorldObserver;

protected:
    void computeOpportunityImpacts() override;

};


#endif //ROBOROBO3_NegociateGymTrainWorldObserver_H
