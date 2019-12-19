/**
 * @file
 * @author Leo Cazenille <leo.cazenille@upmc.fr>
 *
 *
 */

#include <neuralnetworks/EigenMLP.h>
#include <neuralnetworks/NeuralNetwork.h>
#include <sstream>
#include <cmath>
#include <Eigen/Eigen>

using namespace Neural;
using namespace Eigen;

/* --------------------- MLP --------------------- */

EigenMLP::EigenMLP(std::vector<double>& weights,
		unsigned int nbInputs,
		unsigned int nbOutputs,
		bool activeBias,
		bool onlyUseBiasForFirstHiddenLayer,
		double biasValue) :
		LayeredNeuralNetwork(weights, nbInputs, nbOutputs, activeBias, onlyUseBiasForFirstHiddenLayer, biasValue) {
	// ...
}


EigenMLP::EigenMLP(std::vector<double>& weights,
		unsigned int nbInputs,
		unsigned int nbOutputs,
		std::vector<unsigned int>& nbNeuronsPerLayer,
		bool activeBias,
		bool onlyUseBiasForFirstHiddenLayer,
		double biasValue) :
		LayeredNeuralNetwork(weights, nbInputs, nbOutputs, nbNeuronsPerLayer, activeBias, onlyUseBiasForFirstHiddenLayer, biasValue) {
}



EigenMLP::EigenMLP(EigenMLP const& other) : LayeredNeuralNetwork(other) {
	// ...
}

EigenMLP::~EigenMLP() {
	// ...
}


std::string EigenMLP::toString() const {
	return LayeredNeuralNetwork::toString();
}


void EigenMLP::step() {

	VectorXd inputs(_inputs.size());
	for (int i = 0; i < _inputs.size(); i++)
    {
        inputs[i] = _inputs[i];
    }

	// Verify that the number of layers is correct
//	if(_nbNeuronsPerLayer.size() < 2)
//		throw NeuralNetworkException("EigenMLP must have at least 2 layers : input and output");
//	if(_nbNeuronsPerLayer[0] != _inputs.size())
//		throw NeuralNetworkException("nbNeuronsPerLayer has an incorrect number of inputs neurons (first layer)");
//	if(_nbNeuronsPerLayer[_nbNeuronsPerLayer.size() - 1] == 0)
//		throw NeuralNetworkException("nbNeuronsPerLayer has an incorrect number of output neurons (output layer)");

//	unsigned int nbBias = 0;
//	if(_activeBias)
//		nbBias = 1;

    VectorXd tmp = inputs;
	int curMat = 0;
	for (MatrixXd& mat : weightMatrices)
    {
        if (_activeBias && (!_onlyUseBiasForFirstHiddenLayer || curMat == 0))
        {
            tmp.conservativeResize(tmp.size() + 1);
            tmp[tmp.size() - 1] = _biasValue;
        }
        curMat++;
        tmp = mat * tmp;
        for (int i = 0; i < tmp.size(); i++)
        {
            tmp[i] = activation(tmp[i]);
        }

    }

    for (int i = 0; i < _nbOutputs; i++)
    {
        _outputs[i] = tmp[i];
    }
}


unsigned int EigenMLP::computeRequiredNumberOfWeights() {
	unsigned int res = 0;
	unsigned int nbBias = 0;
	if(_activeBias)
		nbBias = 1;

	if(_nbNeuronsPerLayer.size() <= 2) {
		return (_nbInputs + nbBias) * _nbOutputs;
	} else {
		res += (_nbInputs + nbBias) * _nbNeuronsPerLayer[1];
		if(_onlyUseBiasForFirstHiddenLayer)
			nbBias = 0;
		for(size_t i = 1; i < _nbNeuronsPerLayer.size() - 1; i++) {
			res += (_nbNeuronsPerLayer[i] + nbBias) * _nbNeuronsPerLayer[i + 1];
		}
		return res;
	}
}


std::string EigenMLP::getNNTypeName() {
	return "EigenMLP";
}

EigenMLP* EigenMLP::clone() const {
	return new EigenMLP(*this);
}

void EigenMLP::setWeights(std::vector<double> &weights) {
    NeuralNetwork::setWeights(weights);


    weightMatrices.clear();

    int cur = 0;
    for (int i = 0; i < _nbNeuronsPerLayer.size() - 1; i++)
    {
        int nbFrom = _nbNeuronsPerLayer[i];
        int nbTo = _nbNeuronsPerLayer[i+1];

        if (_activeBias && (! _onlyUseBiasForFirstHiddenLayer || i == 0))
        {
            nbFrom++;
        }
        weightMatrices.emplace_back(nbTo, nbFrom);
        MatrixXd& mat = weightMatrices[weightMatrices.size() - 1];
        for (int iTo = 0; iTo < nbTo; iTo++) {
            for (int iFr = 0; iFr < nbFrom; iFr++) {
                mat(iTo, iFr) = _weights[cur++];
            }
        }
    }
    assert (cur == _weights.size());
}

