/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-14
 */

#ifndef ROBOROBO3_LIONSHAREDDATA_H
#define ROBOROBO3_LIONSHAREDDATA_H


class LionSharedData
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
    static int oppDecay;
    static double maxCoop;
    static double b;
    static double meanA;
    static double stdA;
    static bool fakeRobots;
    static double fakeCoef;
    static int logEveryXGen;
    static bool randomFakeCoef;
    static double fakeCoefStd;
    static bool commonKnowledgeReputation;
    static double reputationNoise;
    static bool onlyNforGame;
    static double frictionCoef;
    static double frictionInflexionPoint;
    static double tpProba;
    static double cost;
    static bool optimalPlay;

    static void initSharedData();



};


#endif //ROBOROBO3_LIONSHAREDDATA_H
