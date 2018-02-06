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
    static int takeVideoEveryGeneration;
    static int seeCoopFromDist;
    static bool prisonerDilemma;
    static bool selfAAsInput;

    static void initSharedData();

    static int oppDecay;
    static bool fixRobotNb;
    static bool teleportRobots;
};


#endif //ROBOROBO3_COOPFIXED2SHAREDDATA_H
