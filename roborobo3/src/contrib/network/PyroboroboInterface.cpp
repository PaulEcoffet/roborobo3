//
// Created by Paul Ecoffet on 25/05/2020.
//

#include "contrib/network/PyroboroboInterface.h"
#include "libnpy/npy.h"
#include <iostream>
#include "json/json.hpp"

using json = nlohmann::json;


PyroboroboInterface::PyroboroboInterface(const std::string &ip, unsigned short port) :
        ni(ip, port)
{
}

void PyroboroboInterface::connect(const std::string &ip, unsigned short port)
{
    ni.connect(ip, port);
}

void PyroboroboInterface::initPyroborobo(const int _popsize,
                                         const std::vector<double> &minbounds_obs,
                                         const std::vector<double> &maxbounds_obs,
                                         const std::vector<double> &minbounds_act,
                                         const std::vector<double> &maxbounds_act)
{
    assert(minbounds_act.size() == maxbounds_act.size());
    assert(minbounds_obs.size() == maxbounds_act.size());
    assert(_popsize > 0);

    popsize = _popsize;
    dims_act = minbounds_act.size();
    dims_obs = minbounds_obs.size();


    json params = {
            {"popsize",        _popsize},
            {"min_bounds_obs", minbounds_obs},
            {"max_bounds_obs", maxbounds_obs},
            {"min_bounds_act", minbounds_act},
            {"max_bounds_act", maxbounds_act}
    };
    ni.sendMessage(params.dump());
}

std::vector<double> PyroboroboInterface::getActionFromObservations(const std::vector<double> &obs)
{
    std::stringstream array_as_bytestream;
    unsigned long obs_dims[] = {popsize, dims_obs};
    bool discard_fortran_order;
    std::vector<unsigned long> act_dims;
    std::vector<double> action_vector;

    /* Send observations */
    npy::SaveArrayAsNumpy(array_as_bytestream, false, 2, obs_dims, obs);
    ni.sendMessage(array_as_bytestream.str());

    /* Receive actions */
    std::string action_npy = ni.receiveMessage();
    array_as_bytestream.str(action_npy);
    npy::LoadArrayFromNumpy(array_as_bytestream, act_dims, discard_fortran_order, action_vector);
    return action_vector;
}
