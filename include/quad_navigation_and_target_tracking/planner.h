#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include<chrono>
#include <fstream>
#include <math.h>
#include <string> 
#include <ros/ros.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
//#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <Eigen/Geometry>
//#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/filters/frustum_culling.h>
#include <mav_msgs/Actuators.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <trajectoryGenerationOptimalControl.h>
#include <controller_px4.h>
#include <controller_simulation.h>
#include <ensembleGeneration.h>





using namespace std;
using namespace Eigen;
//std::ofstream outdata;
//Eigen::Matrix4f lidar_pose;

class planner
{
  public: 
    planner(ros::NodeHandle* nodehandle) 
    {
      ROS_INFO("in class constructor of Planner");
      odom_counter = 0;
      pc_counter = 0;
      wp_counter = 0;
      rhp_counter = 0;
      local_goal = {0,0,0,0};
      initialState = VectorXd::Zero(12);
      currentState = VectorXd::Zero(12);
      desiredState = VectorXd::Zero(14);
      goalState = VectorXd::Zero(12);
      localGoal = VectorXd::Zero(12);
      px = VectorXd::Zero(8);
      py = VectorXd::Zero(8);
      pz = VectorXd::Zero(8);
      cogTbase = Eigen::Matrix4d::Identity();
      baseTcoc =  Eigen::Matrix4d::Identity();
      cogTcoc = Eigen::Matrix4d::Identity();
      outdata.open("generated_trajectory.txt", std::ios::out|std::ios::binary);
      outdata1.open("generated_trajectory2.txt", std::ios::out|std::ios::binary);
      outdata2.open("data_for_k.txt", std::ios::out|std::ios::binary);
      getParameters();
      initializeSubscribers(); 
      initializePublishers();
      //std::vector <double> xdes, ydes, zdes, vxdes, vydes, vzdes, axdes, aydes, azdes, traj_global_time;
    }; 
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void finalTrajectory(VectorXd& px, VectorXd& py, VectorXd& pz, Vector4d& yaw_coeff, double& T0, double& T);
    void checkConvergance(VectorXd& goalState, VectorXd& currentState, bool& waypointReached, double& convergance);
  private:     
    ros::NodeHandle nh_;
    void initializeSubscribers(); 
    void initializePublishers();
    void getParameters();

    unsigned int odom_counter, pc_counter, wp_counter, rhp_counter;

    double PI = 3.1416;
    double odom_frequency, pc_frequency, maxR, minR, xFOV, yFOV, mass, maxT, maxV, avgV, maxAngV;
    double hHover, Distance2Goal;
    double delt, T, controlHorizonTime;
    double currentYaw, currentYawrate, desiredYaw, desiredYawrate, controlHorizonYaw, controlHorizonYawrate;
    double tau_rp, tau_yaw;
    double current_time, replanning_starttme;

    ros::Subscriber odom_sub;
    ros::Subscriber pc_sub;
    ros::Publisher cmd_pub, pc_pub, ensemble_pub, best_traj_pub, selected_traj_pub;

    string mode, odom_topic, pc_topic, cmd_topic, pcFrame, camBaseframe;
    
    VectorXd initialState, currentState, desiredState, goalState, px, py, pz, localGoal;
    Eigen::Vector4d yaw_coeff;

    Matrix4d cogTbase, baseTcoc, cogTcoc, dronePose;

    std::vector <double> iPosition, final_goal, kx, kv, kR, kOmega, cgTb, bToc, local_goal;
    std::vector< std::vector<double>> waypointList;
    std::vector <double> xdes, ydes, zdes, vxdes, vydes, vzdes, axdes, aydes, azdes, traj_global_time, yawdes, yawratedes;

    bool waypointReached = false;    
    bool trajectoryAvailable = false;    
    bool replan = false;
    bool RHPmode = true;
    

    std::ofstream outdata, outdata1, outdata2;

    tf::StampedTransform transform;

    pcl::PointCloud<pcl::PointXYZ> traj_cloud, selected_traj_cloud;

    
    
    
    
};
#endif //PLANNER_H
