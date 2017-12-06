/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr> 
 * @date 2017-12-06
 */

#include "contrib/network/PyCMAESInterface.h"
#include "contrib/json/json.hpp"

using json = nlohmann::json;


PyCMAESInterface::PyCMAESInterface(const std::string& ip, unsigned short port)
{
    connect(ip, port);
}

void PyCMAESInterface::connect(const std::string &ip, unsigned short port)
{
    ni.connect(ip, port);
}

std::vector<std::vector<double>> PyCMAESInterface::initCMA(int popsize, int parameters_dimension)
{
    json params = {
            {"popsize", popsize},
            {"nb_weights", parameters_dimension}
    };
    ni.sendMessage(params.dump());
    return getNextGeneration(std::vector<double>());
}

std::vector<std::vector<double>> PyCMAESInterface::getNextGeneration(std::vector<double> fitnesses)
{
    std::vector<std::vector<double>> genomes;

    if (!fitnesses.empty())
    {
        json json_fit = fitnesses;
        ni.sendMessage(json_fit.dump());
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

void PyCMAESInterface::close()
{
    ni.close();
}
