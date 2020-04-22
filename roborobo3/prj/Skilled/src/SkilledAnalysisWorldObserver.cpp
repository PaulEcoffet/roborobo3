/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "Skilled/include/SkilledAnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <Skilled/include/SkilledOpportunity.h>
#include <Skilled/include/SkilledSharedData.h>
#include <Skilled/include/SkilledController.h>
#include <Skilled/include/SkilledWorldModel.h>
#include <Skilled/include/SkilledAnalysisOpportunity.h>
#include <Skilled/include/SkilledWorldObserver.h>
#include "Skilled/include/SkilledAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

SkilledAnalysisWorldObserver::SkilledAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    SkilledSharedData::initSharedData();

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

void SkilledAnalysisWorldObserver::reset()
{
    int i_gen = 0;
    for (const auto &genome : m_genomesJson)
    {
        auto *ctl = dynamic_cast<SkilledController *>(gWorld->getRobot(0)->getController());
        ctl->loadNewGenome(genome);
        ctl->resetFitness();
        for (int cost = 0; cost < 2; cost++)
        {
            int maxrob = (SkilledSharedData::maxTwo) ? 4 : gInitialNumberOfRobots;
            for (int nbonopp = 0; nbonopp < maxrob;)
            {
                for (int j = 0; j <= 20; j++)
                {
                    double owncoop = j * 0.5;
                    for (int k = 0; k <= 20; k++)
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
