/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "Lion/include/LionAnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <Lion/include/LionOpportunity.h>
#include <Lion/include/LionSharedData.h>
#include <Lion/include/LionController.h>
#include <Lion/include/LionWorldModel.h>
#include <Lion/include/LionAnalysisOpportunity.h>
#include <Lion/include/LionWorldObserver.h>
#include "Lion/include/LionAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

LionAnalysisWorldObserver::LionAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    LionSharedData::initSharedData();

    int gen = 0;
    gProperties.checkAndGetPropertyValue("genAnalysis", &gen, true);

    std::string genomePath(gLogDirectoryname + "/genomes_" + std::to_string(gen) + ".txt");
    gProperties.checkAndGetPropertyValue("genomePath", &genomePath, false);
    if (!boost::filesystem::exists(genomePath))
    {
        std::cerr << "The genome file path '" << genomePath << "' does not exist.\n";
        exit(-1);
    }
    std::ifstream genomeFile(genomePath);
    genomeFile >> m_genomesJson;

    m_log.open(gLogDirectoryname + "/analysis_log_" + std::to_string(gen) + ".txt");
    m_log << "ind\tcost\tnbopp\towncoop\tothercoop\tscore\n";

}

void LionAnalysisWorldObserver::reset()
{
    int i_gen = 0;
    for (const auto& genome : m_genomesJson)
    {
        auto *ctl = dynamic_cast<LionController *>(gWorld->getRobot(0)->getController());
        ctl->loadNewGenome(genome);
        ctl->resetFitness();
        for (int cost = 0; cost < 2; cost++)
        {
            for (int nbonopp = 0; nbonopp < gInitialNumberOfRobots; nbonopp++)
            {
                double owncoop = ctl->getCoop(nbonopp);
                for(int k = 0; k <= 20; k++)
                {
                    double partcoop = k * 0.5;
                    double totothercoop = partcoop * nbonopp;
                    double score = ctl->computeScore(cost, nbonopp, owncoop, totothercoop);
                    m_log << i_gen << "\t"
                          << cost << "\t"
                          << nbonopp << "\t"
                          << owncoop << "\t"
                          << partcoop << "\t"
                          << score << "\n";
                }
            }
        }
        i_gen++;
    }
    m_log.close();
    exit(0);
}
