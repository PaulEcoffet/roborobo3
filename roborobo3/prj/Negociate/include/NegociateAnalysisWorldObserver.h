/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_NEGOCIATEANALYSISWORLDOBSERVER_H
#define ROBOROBO3_NEGOCIATEANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <json/json.hpp>
#include <set>
#include "World/World.h"
#include "Agents/Robot.h"

using json = nlohmann::json;


class NegociateAnalysisWorldObserver : public WorldObserver
{
public:
    explicit NegociateAnalysisWorldObserver(World *__world);


    void reset() override;


protected:
    double m_curCoop;
    int m_curnbrob;
    int m_maxrobnb;
    json m_genomesJson;

    int m_curIterationInRep;
    int m_curRep;
    int m_curInd;


    std::ofstream m_log;

    json::iterator m_genomesIt;


    int m_nbIterationPerRep;
    int m_nbRep;
    double m_stepCoop;

    std::set<int> objectsToTeleport;
};


#endif //ROBOROBO3_NEGOCIATEANALYSISWORLDOBSERVER_H
