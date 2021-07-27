#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


using namespace std;
using namespace Eigen;
using std::ofstream;


class controller
{
  public: 
    controller(ros::NodeHandle* nodehandle)
    {
      ROS_INFO("in class constructor of controller");
      getParamters();
      initializeSubscribers(); 
      initializePublishers();

      curr_pose = {0,0,0,0,0,0,0,0};
      global_counter = 0;
    };
    void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& _msg);  
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    Eigen::Vector4d rot2quat(const Eigen::Matrix3d &R);
    Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
    Eigen::Matrix3d quat2rot(const Eigen::Vector4d &q);
  private: 
    double PI = 3.1416;
    Eigen::Vector3d g = {0, 0, 9.8};
    ros::NodeHandle nh_;
    void initializeSubscribers(); 
    void initializePublishers();
    void getParamters();

    ros::Subscriber odom_sub, trajectory_sub, state_sub;
    ros::Publisher cmd_pub, pose_pub; 

    double m, Tc, Hz, f;
    std::vector <double> final_goal, kx, kv;
    //Eigen::Vector3f final_goal, kx, kv;

    string pose_topic, trajectory_topic;

    std::vector <double> curr_pose;
    std::vector <double> xdes, ydes, vxdes, vydes, thetades, wdes, traj_global_time;
    int global_counter;
    bool TrajectoryPublished = false;

    mavros_msgs::State current_mavros_state;
    ros::ServiceClient arming_client, set_mode_client;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::Time start_time;

    std::ofstream outdata;

};

