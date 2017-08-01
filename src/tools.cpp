#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(estimations[0].size());
	VectorXd sum(estimations[0].size());

	sum.fill(0);
	for (int i = 0; i<estimations.size(); i++)
	{
		VectorXd error = estimations[i] - ground_truth[i];
		VectorXd error2 = error.array()*error.array();
		sum += error2;
	}
	rmse = (sum / estimations.size()).array().sqrt();

	//cout <<"size is :" <<estimations.size()<< ground_truth.size() << endl;
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	//check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}
	//compute the Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
	return Hj;
}
