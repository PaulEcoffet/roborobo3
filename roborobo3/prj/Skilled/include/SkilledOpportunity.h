//
// Created by paul on 31/10/17.
//

#ifndef ROBOROBO3_SKILLEDOPPORTUNITY_H
#define ROBOROBO3_SKILLEDOPPORTUNITY_H


#include "World/RoundObject.h"
#include "Skilled/include/SkilledWorldModel.h"
#include <set>

class SkilledWorldModel;

class SkilledOpportunity : public RoundObject
{
public:
    explicit SkilledOpportunity(int __id);

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

    bool isRobotOnOpp(const int id);


protected:
    std::vector<int> nearbyRobotIndexes;
    std::map<int, SkilledWorldModel *> nearbyMap;

    virtual void updateColor();


    double sumCoop(int nbonopp);

};


#endif //ROBOROBO3_SKILLEDOPPORTUNITY_H
