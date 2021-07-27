#ifndef VIOTOCOG_H
#define VIOTOCOG_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"



using namespace std;
using namespace Eigen;
using std::ofstream;


class vio2cog
{
  public: 
    vio2cog(ros::NodeHandle* nodehandle)
    { 
    odom_counter = 0;
    outdata.open("vel_comparison.txt", std::ios::out|std::ios::binary);
    getParameters();
    initializeSubscribers(); 
    initializePublishers();
    };
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);
    Eigen::Vector4d rot2quat(const Eigen::Matrix3d &R);
  private: 
    double PI = 3.1415;
    Eigen::Vector3d g = {0, 0, 9.8};
    Eigen::Vector3d g_sum = {0, 0, 9.8}; // used to get the initial value of gravity based on first 50 imu readings
    ros::NodeHandle nh_;
    void initializeSubscribers(); 
    void initializePublishers();
    void getParameters();

    int odom_counter;
    int Nsamples = 5;
    double odom_frequency, imu_frequency, dt_odom, dt_imu;

    ros::Subscriber odom_sub, pose_sub;
    ros::Publisher odom_pub, mavros_pub, mavros_odom_pub;

    string input_odom, input_pose, output_odom, mavros_topic;

    std::vector <double> _RMtoG, _vGinM, _pGinM, _pCoGinI, _RItoCoG;

    Eigen::Vector3d vGinM, pGinM, pCoGinI, pCoGinM, vCoGinM;
    std::vector<double> vx, vy, vz;
    Eigen::Matrix3d RMtoG, RItoCoG, RGtoI_dot, RGtoI_previous, RMtoCoG;
    std::ofstream outdata;

};

#endif //VIOTOCOG_H

