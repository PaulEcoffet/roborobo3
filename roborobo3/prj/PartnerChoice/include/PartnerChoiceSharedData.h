/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-31
 */

#ifndef ROBOROBO3_PARTNERCHOICESHAREDDATA_H
#define ROBOROBO3_PARTNERCHOICESHAREDDATA_H


class PartnerChoiceSharedData
{
public:
    static int evaluationTime;
    static int genomeLog;
    static int controllerType;
    static double maxTranslationalValue;
    static double maxRotationalVelocity;
    static int nbHiddenLayers;
    static int nbNeuronsPerHiddenLayer;
    static double maxCoop;
    static double constantA;
    static double constantK;
    static int nbCoopStep;
    static double sigma;
    static int nbEvaluationsPerGeneration;
    static int takeVideoEveryGeneration;
};


#endif //ROBOROBO3_PARTNERCHOICESHAREDDATA_H
