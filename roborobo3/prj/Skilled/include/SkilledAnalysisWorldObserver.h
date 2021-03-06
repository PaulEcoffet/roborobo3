/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2018-01-08
 */

#ifndef ROBOROBO3_SKILLEDANALYSISWORLDOBSERVER_H
#define ROBOROBO3_SKILLEDANALYSISWORLDOBSERVER_H


#include <Observers/WorldObserver.h>
#include <json/json.hpp>
#include <set>
#include <gzstream.h>
#include "World/World.h"
#include "Agents/Robot.h"

using json = nlohmann::json;


class SkilledAnalysisWorldObserver : public WorldObserver
{
public:
    explicit SkilledAnalysisWorldObserver(World *__world);


    void reset() override;


protected:
    json m_genomesJson;
    ogzstream m_log;


};


#endif //ROBOROBO3_SKILLEDANALYSISWORLDOBSERVER_H
