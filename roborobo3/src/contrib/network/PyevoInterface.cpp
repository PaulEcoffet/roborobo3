/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2017-12-06
 */

#include <contrib/network/PyevoInterface.h>

#include "network/PyevoInterface.h"
#include "json/json.hpp"

using json = nlohmann::json;


PyevoInterface::PyevoInterface(const std::string& ip, unsigned short port)
{
    connect(ip, port);
}

void PyevoInterface::connect(const std::string &ip, unsigned short port)
{
    ni.connect(ip, port);
}

std::vector<std::vector<double>> PyevoInterface::initCMA(int popsize, int parameters_dimension, std::vector<double>&minbounds,
                                                         std::vector<double>&maxbounds, std::vector<double>&minguess,
                                                         std::vector<double>&maxguess)
{
    json params = {
            {"popsize", popsize},
            {"nb_weights", parameters_dimension},
            {"min_bounds", minbounds},
            {"max_bounds", maxbounds},
            {"min_guess", minguess},
            {"max_guess", maxguess}
    };
    ni.sendMessage(params.dump());
    return getNextGeneration(std::vector<std::vector<double>>(), std::vector<double>());
}

std::vector<std::vector<double>> PyevoInterface::initCMA(int popsize, int parameters_dimension) {
    std::vector<double> minbounds(popsize, -5);
    std::vector<double> maxbounds(popsize, 5);

    return PyevoInterface::initCMA(popsize, parameters_dimension, minbounds, maxbounds, minbounds, maxbounds);
}

std::vector<std::vector<double>> PyevoInterface::getNextGeneration(std::vector<std::vector<double>>individuals, std::vector<double> fitnesses)
{
    std::vector<std::vector<double>> genomes;

    if (!fitnesses.empty() and !individuals.empty())
    {
        json json_fit = fitnesses;
        json json_ind = individuals;
        json json_both;
        json_both["fitness"] = json_fit;
        json_both["ind"] = individuals;

        ni.sendMessage(json_both.dump());
    }
    std::string receivedJson = ni.receiveMessage();
    if (receivedJson.empty())
    {
        return genomes;
    }
    json json_genome = json::parse(receivedJson);
    for (const auto &genome : json_genome)
    {
        std::vector<double> cur_gen = genome;
        genomes.push_back(cur_gen);
    }
    return genomes;
}

void PyevoInterface::close()
{
    ni.close();
}


