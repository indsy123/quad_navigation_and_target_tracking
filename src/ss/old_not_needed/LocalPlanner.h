#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <math.h>
#include <string> 
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Geometry>
#include <pcl/filters/frustum_culling.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace Eigen;
//std::ofstream outdata;
//Eigen::Matrix4f lidar_pose;

class localplanner
{
    public: 
      localplanner(ros::NodeHandle* nodehandle) 
      {
        ROS_INFO("in class constructor of controller");
        getParamters();
        initializeSubscribers(); 
        initializePublishers();

        pose_msg_counter  = 0;
        ch_point = {0,0,0,0,0,0,0,0};
        curr_pose = {0,0,0,0,0,0,0,0};
        ContinueReplanning = true;
        FinalGoalinFOV = false;
      }; 
      void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
      void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
      void PublishTrajectory(const sensor_msgs::PointCloud2ConstPtr& cloud, VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t, VectorXd& XXddot, VectorXd& YYddot);
      void publishTransform(const nav_msgs::Odometry::ConstPtr& msg);
    private: 
      ros::NodeHandle nh_;
      void initializeSubscribers(); 
      void initializePublishers();
      void getParamters();

      ros::Subscriber odom_sub, pc_sub;
      ros::Publisher traj_pub, best_traj_pub, ensamble_pub;

      string pointcloud_topic, pose_topic;
      double odometry_frequency, lidar_frequency, av_speed, SensorRangeMax, SensorRangeMin, SensorFOV;
      vector<double> ch_point;
      vector<double> curr_pose; 
      vector<double> final_goal; 

      unsigned int pose_msg_counter;
      double ch_time, delt;
      int intervals;
      double Distance2Goal;
      bool ContinueReplanning;
      bool FinalGoalinFOV;
      pcl::PointCloud<pcl::PointXYZ> traj_cloud;

      // writing data to a file
      std::ofstream outdata;
      
    
};
#endif //LOCALPLANNER_H
