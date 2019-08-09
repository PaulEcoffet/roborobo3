/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "Negociate/include/NegociateAnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <Negociate/include/NegociateOpportunity.h>
#include <Negociate/include/NegociateSharedData.h>
#include <Negociate/include/NegociateController.h>
#include <Negociate/include/NegociateWorldModel.h>
#include <Negociate/include/NegociateAnalysisOpportunity.h>
#include <Negociate/include/NegociateWorldObserver.h>
#include "Negociate/include/NegociateAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

NegociateAnalysisWorldObserver::NegociateAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    NegociateSharedData::initSharedData();

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
    m_genomesIt = m_genomesJson.begin();

    m_log.open(gLogDirectoryname + "/analysis_log_" + std::to_string(gen) + ".txt");
    m_log << "ind\towncoop\tother\taccept\n";

    gMaxIt = -1;
}

void NegociateAnalysisWorldObserver::reset()
{
    const double maxCoop = NegociateSharedData::maxCoop;
    int curind = 0;
    while (m_genomesIt < m_genomesJson.end())
    {
        m_log << std::flush;
        auto *ctl = dynamic_cast<NegociateController *>(gWorld->getRobot(0)->getController());
        auto *wm = dynamic_cast<NegociateWorldModel *>(gWorld->getRobot(0)->getWorldModel());
        ctl->loadNewGenome(*m_genomesIt);
        ctl->resetFitness();
        for (int icoop = 0; icoop < 20; icoop++)
        {
            double owncoop = (double) icoop / (20 - 1) * maxCoop;
            for (int iother = 0; iother < 20; iother++)
            {
                double othercoop = (double) iother / (20 - 1) * maxCoop;
                wm->onOpportunity = true;
                wm->arrival = 1;
                wm->lastOwnInvest.clear();
                wm->lastTotalInvest.clear();
                wm->nbOnOpp = 2;
                wm->appendOwnInvest(owncoop);
                wm->appendTotalInvest(othercoop);
                bool accept = ctl->acceptPlay();
                m_log << curind << "\t"
                      << owncoop << "\t"
                      << othercoop << "\t"
                      << accept << "\n";
            }
        }
        m_log.flush();
        curind++;
        m_genomesIt++;
    }
    m_genomesIt++;
    m_log.close();
    exit(0);
}


