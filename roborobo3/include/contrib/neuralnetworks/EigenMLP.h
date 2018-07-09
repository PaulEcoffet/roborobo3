/**
 * @file
 * @author Leo Cazenille <leo.cazenille@upmc.fr>
 *
 *
 */

#ifndef EIGENMLP_H
#define EIGENMLP_H
#include <neuralnetworks/LayeredNeuralNetwork.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace Neural {

	/**
	 * A basic Multi-Layers Perceptron
	 * @author Leo Cazenille <leo.cazenille@upmc.fr>
	 */
	class EigenMLP : public LayeredNeuralNetwork {

		protected:

			/**
			 * {@InheritDoc}
			 */
			virtual unsigned int computeRequiredNumberOfWeights();
			std::vector<Eigen::MatrixXd> weightMatrices;



		public:

			// -+-+-  Constructors/Destructors  -+-+- //

			EigenMLP(std::vector<double>& weights, unsigned int nbInputs, unsigned int nbOutputs, bool activeBias = false, bool onlyUseBiasForFirstHiddenLayer = false, double biasValue = 1.0);
			EigenMLP(unsigned int nbInputs, unsigned int nbOutputs, bool activeBias = false, bool onlyUseBiasForFirstHiddenLayer = false, double biasValue = 1.0);
			EigenMLP(std::vector<double>& weights, unsigned int nbInputs, unsigned int nbOutputs, std::vector<unsigned int>& nbNeuronsPerHiddenLayer, bool activeBias = false, bool onlyUseBiasForFirstHiddenLayer = false, double biasValue = 1.0);
			/** Deep Copy constructor */
			EigenMLP(EigenMLP const& other);
			virtual ~EigenMLP();


			// -+-+-  Main Methods  -+-+- //

			/**
			 * {@InheritDoc}
			 */
			virtual EigenMLP* clone() const;

			/**
			 * {@InheritDoc}
			 */
			virtual std::string toString() const;

			/**
			 * {@InheritDoc}
			 */
			virtual void step();

			virtual void setWeights(std::vector<double>& weights);

			/**
			 * Return a string identifying this class
			 */
			static std::string getNNTypeName();

	};

}


#endif

