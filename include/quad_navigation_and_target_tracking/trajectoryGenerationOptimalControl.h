#ifndef POLYNOMIAL_TRAJECTORY_H
#define POLYNOMIAL_TRAJECTORY_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <gsl/gsl_poly.h>
#include <list>
#include <stack>
#include <math.h>
#include <string> 
#include <iostream>
#include <fstream>

using std::vector;
using namespace Eigen;

class OptimalControlTrajectory
{ 
  public: 
    OptimalControlTrajectory() 
    {};
    Eigen::VectorXd linspace(double& start, double& end, const int& num);
    void yawCoeff(Vector2d& yaw_ic, Vector2d& yaw_fc, double& T, Vector4d& yaw_coeff);
    void getOptimalT(VectorXd& start_point, VectorXd& end_point, double& avgV, double& T);
    void minSnapCoeff(VectorXd& start_point, VectorXd& end_point, double& T, VectorXd& coefficients);
    void generateTrajectory(VectorXd& start_point, VectorXd& end_point, double& maxVelocity,
          double& maxThrust, double& maxAngularVelocity, double& mass, VectorXd& px, VectorXd& py, VectorXd& pz, double& T, 
          bool& badTrajectory);
    void getControlHorizonPoint(VectorXd &px,VectorXd &py, VectorXd &pz, double& t, VectorXd& p);

  private:
    int trajectory_counter = 0;
    double PI = 3.1416;
    //std::ofstream outdata;

};
#endif //POLYNOMIAL_TRAJECTORY_H
