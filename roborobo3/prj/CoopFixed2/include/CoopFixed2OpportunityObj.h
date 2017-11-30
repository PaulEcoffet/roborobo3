/*
 * MovingObject2Max.h
 *
 *  Created on: 9 oct. 2017
 *      Author: Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
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
	/**
	 * The current set of NearbyRobots that arrived during this turn. It may not contain all the robots that arrived
	 * on the current step.
	 */
	std::set<int> _curNearbyRobots;

    /// Amount of time before the robots are allowed to leave the Coop Opportunity
    int _lockRemainingTime = 0;

    /**
     * All the robots that were present last turn. It's a fixed value and should be used for computation, instead of
     * _curNearbyRobots.
     */
    std::set<int> _prevNearbyRobots;



public:
    int getLockRemainingTime() const;

    void setLockRemainingTime(int lockRemainingTime);

    void decrementLockRemainingTime();


    explicit CoopFixed2OpportunityObj( int __id );
	virtual ~CoopFixed2OpportunityObj();

    /**
     * Callback triggered by the Agent when it collides with the opportunity. We register the Agent if there is less
     * than two agent that were present last turn. If there were two agents present last turn, then we register this
     * agent only if it was amongst the two previous agents. Otherwise, we add him to the teleport list of the
     * CoopFixed2WorldObserver.
     *
     * @param __idAgent The id of the Agent who bumped into the cooperation opportunity
     * @param __speed the speed at which the agent bumped into the opportunity. Discarded here.
     */
	void isPushed( int __idAgent, std::tuple<double, double> __speed ) override; // callback


    /**
     * @return The set of the Agent ID that bumped into the opportunity the step before.
     */
    std::set<int> getNearbyRobots();

    /*
     * Put the _curNearbyRobots in _prevNearbyRobots, should be called once all the robots of the turn has played.
     */
    void clearNearbyRobots();


    void step() override;

    /**
     * @return the number of robots on this opportunity at the previous step.
     */
    int getNbNearbyRobots();

	std::string inspect(std::string prefix="") override;
};

#endif /* PRJ_COOPFIXED2_SRC_OPPORTUNITYOBJ_H_ */
