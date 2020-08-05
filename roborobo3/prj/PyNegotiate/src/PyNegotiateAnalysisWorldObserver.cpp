/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#include "PyNegotiate/include/PyNegotiateAnalysisWorldObserver.h"

#include <RoboroboMain/main.h>
#include <PyNegotiate/include/PyNegotiateOpportunity.h>
#include <PyNegotiate/include/PyNegotiateSharedData.h>
#include <PyNegotiate/include/PyNegotiateController.h>
#include <PyNegotiate/include/PyNegotiateWorldModel.h>
#include <PyNegotiate/include/PyNegotiateAnalysisOpportunity.h>
#include <PyNegotiate/include/PyNegotiateWorldObserver.h>
#include "PyNegotiate/include/PyNegotiateAnalysisWorldObserver.h"
#include "json/json.hpp"

using json = nlohmann::json;

PyNegotiateAnalysisWorldObserver::PyNegotiateAnalysisWorldObserver(World *__world) : WorldObserver(__world)
{
    PyNegotiateSharedData::initSharedData();

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
    std::cout << *m_genomesIt << "\n";

    m_log.open((gLogDirectoryname + "/analysis_log_" + std::to_string(gen) + ".txt.gz").c_str());
    m_log << "ind\towncoop\tother\taccept\n";

    gMaxIt = -1;
}

void PyNegotiateAnalysisWorldObserver::reset()
{
    const double maxCoop = PyNegotiateSharedData::maxCoop;
    int gen = 0;
    gProperties.checkAndGetPropertyValue("genAnalysis", &gen, true);

    std::ofstream genlog(gLogDirectoryname + "/coop_" + std::to_string(gen) + ".txt");
    genlog << "ind\tcoop\n";
    int curind = 0;
    while (m_genomesIt < m_genomesJson.end())
    {
        m_log << std::flush;
        auto *ctl = dynamic_cast<PyNegotiateController *>(gWorld->getRobot(0)->getController());
        auto *wm = dynamic_cast<PyNegotiateWorldModel *>(gWorld->getRobot(0)->getWorldModel());
        /* TODO REIMPLEMENT LOAD GENE ctl->loadNewGenome(*m_genomesIt); */
        ctl->resetFitness();
        double gencoop = wm->getCoop(true);
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
                wm->_cooperationLevel = owncoop;
                wm->appendTotalInvest(othercoop);
                bool accept = ctl->acceptPlay();
                m_log << curind << "\t"
                      << owncoop << "\t"
                      << othercoop << "\t"
                      << accept << "\n";
            }
        }
        genlog << curind << "\t" << gencoop << "\n";
        m_log.flush();
        curind++;
        m_genomesIt++;
    }
    m_genomesIt++;
    m_log.close();
    genlog.close();
    exit(0);
}


