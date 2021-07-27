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


class controller_px4
{
  public: 
    controller_px4()
    {
      control_counter = 0;
    };
    Eigen::Vector4d rot2quat(const Eigen::Matrix3d &R);
    Eigen::Matrix3d quat2rot(const Eigen::Vector4d &q);
    void calculateCommands(const nav_msgs::Odometry::ConstPtr& msg, VectorXd& desiredState, std::vector<double>& kx, 
                           std::vector<double>& kv, double& tau_rp, double& tau_yaw, double& maxThrust, double& mass, VectorXd& w);
  private: 
    double PI = 3.1416;
    Eigen::Vector3d g = {0, 0, 9.8};
    double previous_phi = 0;
    int control_counter;

};

