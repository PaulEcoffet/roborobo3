//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_LIONOPPORTUNITY_H
#define ROBOROBO3_LIONOPPORTUNITY_H


#include "World/RoundObject.h"
#include "Lion/include/LionWorldModel.h"
#include <set>

class LionWorldModel;

class LionOpportunity : public RoundObject
{
public:
    explicit LionOpportunity(int __id);

    void step() override;
    void reset();


    void isWalked(int id) override;

    virtual double getCurInv() const;

    int countCurrentRobots();
    void removeRobot(int id);


    virtual void kill();

    std::string inspect(std::string prefix) override;

    double curInv = 0;
    double curA = 5;
    double ifNewPartInv = 0;
    int lifeid;
    double lifeExpectancy;
    double getIfNewPartInv() const;



protected:
    std::vector<int> nearbyRobotIndexes;
    std::map<int, LionWorldModel*> nearbyMap;

    virtual void updateColor();


    double sumCoop(int i);
};


#endif //ROBOROBO3_LIONOPPORTUNITY_H
