#ifndef _LowPassFilter_hpp_
#define _LowPassFilter_hpp_

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
class LowPassFilter{
public:
	//constructors
	LowPassFilter();
	//functions
	Eigen::VectorXd update(Eigen::VectorXd input);
	Eigen::VectorXd update(Eigen::VectorXd input, float deltaTime, Eigen::VectorXd cutoffFrequency);
	//get and configure funtions
	Eigen::VectorXd getOutput() const{return output;}
	void reconfigureFilter(float deltaTime, Eigen::VectorXd cutoffFrequency,int dof);
private:
	Eigen::VectorXd output;
	Eigen::MatrixXd EPOW;
	float ePow;
	int dof;
};

#endif //_LowPassFilter_hpp_