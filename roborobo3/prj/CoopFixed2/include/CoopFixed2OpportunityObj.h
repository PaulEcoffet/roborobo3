/*
 * MovingObject2Max.h
 *
 *  Created on: 9 oct. 2017
 *      Author: Paul Ecoffet
 */

#ifndef PRJ_COOPFIXED2_SRC_OPPORTUNITYOBJ_H_
#define PRJ_COOPFIXED2_SRC_OPPORTUNITYOBJ_H_

#include <set>
#include "World/RoundObject.h"
#include "World/World.h"
#include "CoopFixed2WorldObserver.h"

/**
 * An Opportunity Object which can be only touched by two robots maximum. It also prevents the first agent who come to
 * exit the cooperation opportunity before another agent come.
 * See CoopFixed2Controller for additional informations.
 */
class CoopFixed2OpportunityObj: public RoundObject {
protected:
	std::set<int> _nearbyRobots; // robots that are in the footprint in this iteration

    /// Amount of time before the robots are allowed to leave the Coop Opportunity
    int _lockRemainingTime = 0;

    /// The size of the memory
	static constexpr int _memorySize = 20;

	/// remember the total cooperation investment given to the object in the last few turns.
    /// This reduces the noise in investment transmitted to the agents
	double _totalInvestment[_memorySize];

    std::set<int> _prevNearbyRobots;

    //CoopFixed2WorldObserver* _worldObserver;


public:
    int getLockRemainingTime() const;

    void setLockRemainingTime(int lockRemainingTime);

    void decrementLockRemainingTime();


    explicit CoopFixed2OpportunityObj( int __id );
	virtual ~CoopFixed2OpportunityObj();


	void isPushed( int __idAgent, std::tuple<double, double> __speed ) override; // callback


    std::set<int> getNearbyRobots();

    void clearNearbyRobots();

    void step() override;

    int getNbNearbyRobots();
};

#endif /* PRJ_COOPFIXED2_SRC_OPPORTUNITYOBJ_H_ */
