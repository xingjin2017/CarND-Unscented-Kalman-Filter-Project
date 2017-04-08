#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::endl;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Invalid estimations of size " << estimations.size() << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd d = estimations[i] - ground_truth[i];
    d = d.array() * d.array();
    rmse = rmse + d;
  }

  //calculate the mean
  rmse = rmse / estimations.size();
        
  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}
