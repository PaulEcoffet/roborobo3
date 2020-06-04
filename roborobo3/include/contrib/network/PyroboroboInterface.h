//
// Created by Paul Ecoffet on 25/05/2020.
//

#ifndef ROBOROBO3_PYROBOROBOINTERFACE_H
#define ROBOROBO3_PYROBOROBOINTERFACE_H

#include <network/NetworkInterface.h>
#include <string>
#include <vector>

class PyroboroboInterface
{
public:
    PyroboroboInterface() = default;

    PyroboroboInterface(const std::string &ip, unsigned short port);

    ~PyroboroboInterface()
    {
        close();
    };

    /**
     * Connect the Pyroborobo Gym to the python server with at `ip`:`port`.
     * @param ip
     * @param port
     */
    void connect(const std::string &ip, unsigned short port);

    void initPyroborobo(const int _popsize,
                        const std::vector<double> &minbounds_obs, const std::vector<double> &maxbounds_obs,
                        const std::vector<double> &minbounds_act, const std::vector<double> &maxbounds_act);

    std::vector<double> getActionFromObservations(const std::vector<double> &obs);

    void close()
    {
        ni.close();
    };


protected:
    size_t dims_obs = 0;
    size_t dims_act = 0;
    size_t popsize = 0;
    network::NetworkInterface ni;
};


#endif //ROBOROBO3_PYROBOROBOINTERFACE_H
