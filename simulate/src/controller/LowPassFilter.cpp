
#include "LowPassFilter.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#define ERROR_CHECK (true)

#if ERROR_CHECK
#include <iostream>
#endif

LowPassFilter::LowPassFilter()
{

}

Eigen::VectorXd LowPassFilter::update(Eigen::VectorXd input){
	
	return output += EPOW*(input - output);
}

Eigen::VectorXd LowPassFilter::update(Eigen::VectorXd input, float deltaTime, Eigen::VectorXd cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency,dof); //Changes ePow accordingly.
	return output += EPOW*(input - output) ;
}

void LowPassFilter::reconfigureFilter(float deltaTime, Eigen::VectorXd cutoffFrequency,int DOF){
	#if ERROR_CHECK
	if (deltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	
	#endif
	
	output = Eigen::VectorXd::Zero(DOF);
	EPOW = Eigen::MatrixXd::Identity(DOF,DOF);
	
	for(int i=0;i<DOF;i++)
	{
		EPOW(i,i) = 1-exp(-deltaTime * 2 * M_PI * cutoffFrequency(i));
	}
	this->EPOW = EPOW;
	this -> dof = DOF;
}