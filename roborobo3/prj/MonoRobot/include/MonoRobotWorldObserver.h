/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef MONOROBOTWORLDOBSERVER_H
#define MONOROBOTWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MonoRobot/include/MonoRobotSharedData.h"

//class World;

class MonoRobotWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _evaluationCount;
    int _evaluationItCount;
    
    std::pair<int, double> _objectProperties[4];
    std::vector<int> _objectOrder;
    Uint8 _landmarkColors[4][3] = { {0x00, 0x99, 0xFF}, {0xFF, 0x80, 0x00}, {0xFF, 0x69, 0xB4}, {0xFF, 0x00, 0x00} };

    std::ofstream _logFile;
    LogManager *_logManager; // Our own little logfile
    
public:
    MonoRobotWorldObserver(World *world);
    ~MonoRobotWorldObserver();
    
    
    virtual void reset();
    virtual void step();
    void stepEvaluation( bool __newGeneration );
    void resetObjects(); // reset which objects are active, etc.
    void resetLandmarks();
    
    virtual int getGenerationItCount() { return _evaluationItCount; }
    
    std::vector<int> getObjectOrder() { return _objectOrder; }
    
    std::pair<int, double> getObjectProperties(int __obj) { return _objectProperties[__obj]; }

};

#endif
