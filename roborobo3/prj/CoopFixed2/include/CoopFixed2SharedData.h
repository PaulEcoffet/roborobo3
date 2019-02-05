/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#ifndef ROBOROBO3_COOPFIXED2SHAREDDATA_H
#define ROBOROBO3_COOPFIXED2SHAREDDATA_H


class CoopFixed2SharedData
{
public:
    static int evaluationTime;
    static int nbEvaluationsPerGeneration;
    static int genomeLog;
    static int controllerType;
    static double maxTranslationalValue;
    static double maxRotationalVelocity;
    static int nbHiddenLayers;
    static int nbNeuronsPerHiddenLayer;
    static bool takeVideo;
    static int seeCoopFromDist;
    static bool prisonerDilemma;
    static bool selfAAsInput;
    static int oppDecay;
    static bool fixRobotNb;
    static bool teleportRobots;
    static double maxCoop;
    static double b;
    static double meanA;
    static double stdA;
    static bool tpToNewObj;
    static bool totalInvAsInput;
    static bool ownInvAsInput;
    static bool arrivalAsInput;
    static bool onlyOtherInTotalInv;
    static bool fakeRobots;
    static int memorySize;
    static double d;
    static double fakeCoef;
    static bool reputation;
    static int logEveryXGen;
    static bool atLeastTwo;
    static bool reverseCoopOutput;
    static bool splitNetwork;
    static bool randomFakeCoef;
    static double fakeCoefStd;
    static bool smartTeleport;
    static bool punishment;
    static bool punishmentAsInput;
    static bool partnerControl;
    static double sigma;
    static bool fixCoop;
    static bool commonKnowledgeReputation;
    static double reputationNoise;
    static bool onlyNforGame;
    static double frictionCoef;
    static double frictionInflexionPoint;
    static int fitnessUnlockedIter;

    static void initSharedData();


};


#endif //ROBOROBO3_COOPFIXED2SHAREDDATA_H
