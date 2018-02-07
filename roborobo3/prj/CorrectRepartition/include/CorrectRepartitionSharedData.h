/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-31
 */

#ifndef ROBOROBO3_CORRECTREPARTITIONSHAREDDATA_H
#define ROBOROBO3_CORRECTREPARTITIONSHAREDDATA_H


class CorrectRepartitionSharedData
{
public:
    static int evaluationTime;
    static int genomeLog;
    static int controllerType;
    static double maxTranslationalValue;
    static double maxRotationalVelocity;
    static int nbHiddenLayers;
    static int nbNeuronsPerHiddenLayer;
    static int nbEvaluationsPerGeneration;
    static int takeVideoEveryGeneration;

    static void initSharedData();

};


#endif //ROBOROBO3_CORRECTREPARTITIONSHAREDDATA_H
