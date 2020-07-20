//
// Created by paul on 02/11/17.
//

#include <RoboroboMain/main.h>
#include <Skilled/include/SkilledAnalysisWorldObserver.h>
#include <Skilled/include/SkilledTournamentWorldObserver.h>
#include "Skilled/include/SkilledWorldObserver.h"
#include "Skilled/include/SkilledWorldModel.h"
#include "Skilled/include/SkilledAgentObserver.h"
#include "Skilled/include/SkilledController.h"
#include "Config/SkilledConfigurationLoader.h"

WorldObserver *SkilledConfigurationLoader::make_WorldObserver(World *wm) {
    bool analysis = false;
    bool tournament = false;
    gProperties.checkAndGetPropertyValue("analysis", &analysis, false);
    gProperties.checkAndGetPropertyValue("tournament", &tournament, false);
    if (tournament) {
        return new SkilledTournamentWorldObserver(wm);
    }
    if (analysis) {
        return new SkilledAnalysisWorldObserver(wm);
    } else {
        return new SkilledWorldObserver(wm);
    }
}

RobotWorldModel *SkilledConfigurationLoader::make_RobotWorldModel() {
    return new SkilledWorldModel();
}

AgentObserver *SkilledConfigurationLoader::make_AgentObserver(RobotWorldModel *wm) {
    return new SkilledAgentObserver(wm);
}

Controller *SkilledConfigurationLoader::make_Controller(RobotWorldModel *wm) {
    return new SkilledController(wm);
}



