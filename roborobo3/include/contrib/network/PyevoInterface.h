/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2017-12-06
 */

#ifndef ROBOROBO3_PYCMAESINTERFACE_H
#define ROBOROBO3_PYCMAESINTERFACE_H


#include <network/NetworkInterface.h>
#include <string>
#include <vector>

class PyevoInterface
{
public:
    PyevoInterface() = default;
    PyevoInterface(const std::string& ip, unsigned short port);

    /**
     * Connect the PyCMAESInterface to the python server with at `ip`:`port`.
     * @param ip
     * @param port
     */
    void connect(const std::string& ip, unsigned short port);

    std::vector<std::vector<double>> initCMA(int popsize, int parameters_dimension);

    /**
     * Init the CMAStrategy of the python server with a population size of `popsize` in which each individual is
     * parametrized by `parameters_dimension` values.
     *
     * It returns the initial genomes sent by the PyCMAES server.
     *
     * @param popsize The size of the population
     * @param parameters_dimension The number of parameters for each individual
     * @return A vector of size `popsize` of the initial genomes for each individual
     */
    std::vector<std::vector<double>>
    initCMA(int popsize, int parameters_dimension, const std::vector<double> &minbounds,
            const std::vector<double> &maxbounds, const std::vector<double> &minguess,
            const std::vector<double> &maxguess);

    /**
     * Return the next generation of the population based on the fitnesses of the current generation.
     *
     * The fitnesses must be in the same order as the genomes returned by the previous call of getNextGeneration or
     * initCMA.
     *
     * @param fitnesses The fitnesses of the current generation, in the same order as the previous call of
     * getNextGeneration.
     * @return A vector the new genomes for each individual
     */
    std::vector<std::vector<double>>
    getNextGeneration(const std::vector<std::vector<double>> &individuals, const std::vector<double> &fitnesses);

    /**
     * Close the connection between the PyCMAES server and roborobo.
     */
    void close();

private:
    network::NetworkInterface ni;
};


#endif //ROBOROBO3_PYCMAESINTERFACE_H
