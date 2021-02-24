/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef DUMMYWORLDOBSERVER_H
#define DUMMYWORLDOBSERVER_H

#include <core/RoboroboMain/roborobo.h>
#include "Observers/WorldObserver.h"
#include <memory>

class DummyWorldObserver : public WorldObserver
{
public:
    DummyWorldObserver(std::shared_ptr<World> world);
    ~DummyWorldObserver();
    
    void reset();
    void stepPre();
    void stepPost();

};

#endif
