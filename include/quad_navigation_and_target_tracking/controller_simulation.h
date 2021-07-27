#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/JointState.h>
//#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
//#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>


using namespace std;
using namespace Eigen;
using std::ofstream;


class controller_simulation
{
  public: 
    controller_simulation()
    {
      J << 0.0347563, 0, 0, 0, 0.0458929, 0, 0, 0, 0.0977;
      control_counter = 0;
    };
    Eigen::Vector4d rot2quat(const Eigen::Matrix3d &R);
    Eigen::Matrix3d quat2rot(const Eigen::Vector4d &q);
    void calculateCommands(const nav_msgs::Odometry::ConstPtr& msg, VectorXd& desiredState, std::vector<double>& kx, 
                           std::vector<double>& kv,std::vector<double>& kr, std::vector<double>& kw, double& mass, VectorXd& w);
  private: 
    double PI = 3.1416;
    Eigen::Vector3d g = {0, 0, 9.8};
    Eigen::Matrix3d J, Rd_previous;
    double tau_t = 0.00000854858;
    double tau_m = 0.016;
    double cg2cor = 0.215;
    double previous_phi = 0;
    int control_counter;

};

