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

    m_log.open((gLogDirectoryname + "/analysis_log_" + std::to_string(gen) + ".txt.gz").c_str());
    m_log << "ind\tcost\tnbopp\towncoop\tothercoop\tscore\n";

}

void LionAnalysisWorldObserver::reset()
{
    int i_gen = 0;
    double maxcoop = LionSharedData::maxCoop;
    int nbstep = ceil(maxcoop / 0.5);
    for (const auto &genome : m_genomesJson)
    {
        auto *ctl = dynamic_cast<LionController *>(gWorld->getRobot(0)->getController());
        ctl->loadNewGenome(genome);
        ctl->resetFitness();
        for (int cost = 0; cost < 2; cost++)
        {
            int maxrob = (LionSharedData::maxTwo) ? 4 : gInitialNumberOfRobots;
            for (int nbonopp = 0; nbonopp < maxrob;)
            {
                    double owncoop = ctl->getCoop(nbonopp, true);
                    for (int k = 0; k <= nbstep; k++)
                    {
                        double partcoop = ((double) k / nbstep) * maxcoop;
                        double totothercoop = partcoop * nbonopp;
                        double score = ctl->computeScore(cost, nbonopp, owncoop, totothercoop);
                        m_log << i_gen << "\t"
                              << cost << "\t"
                              << nbonopp << "\t"
                              << owncoop << "\t"
                              << partcoop << "\t"
                              << score << "\n";
                    }
                if (nbonopp < 10)
                {
                    nbonopp++;
                }
                else if (nbonopp < 100)
                    nbonopp += 10;
                else
                    nbonopp += 100;
            }
        }
        i_gen++;
    }
    m_log.close();
    exit(0);
}
