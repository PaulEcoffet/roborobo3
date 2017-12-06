/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-10-31
 */

#ifndef ROBOROBO3_FASTWANDERERSHAREDDATA_H
#define ROBOROBO3_FASTWANDERERSHAREDDATA_H


class FastWandererSharedData
{
public:
    static int evaluationTime;
    static int controllerType;
    static double maxTranslationalValue;
    static double maxRotationalVelocity;
    static int nbHiddenLayers;
    static int nbNeuronsPerHiddenLayer;


    static void initSharedData();

};


#endif //ROBOROBO3_FASTWANDERERSHAREDDATA_H
