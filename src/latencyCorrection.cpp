#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Geometry>



ros::Publisher odom_pub;
ros::Publisher mavros_pub; 
double lpf;
//ros::Publisher pose_pub;
//ros::Publisher correctpose_pub;
using namespace std;
using namespace Eigen;
//std::ofstream outdata;

class latencyCorrection
{
    public: 
      latencyCorrection()
      {
        ROS_INFO("in class constructor of latency correction script");
      };

      void cloudposeCb(const nav_msgs::Odometry::ConstPtr& msg);
      void localposeCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
      Eigen::Vector4d rot2quat(const Eigen::Matrix3d &R);
      Eigen::Matrix3d quat2rot(const Eigen::Vector4d &q);     
      
    private: 
      std::pair<double, Eigen::Matrix4d> cloudpose;
      std::pair<double, Eigen::Matrix4d> lp;
      std::vector<std::pair<double, Eigen::Matrix4d>> localpose;
      std::vector<double> localpose_times;
      std::pair<Eigen::Vector3d, Eigen::Vector3d> X_corrected;
};


Eigen::Vector4d latencyCorrection::rot2quat(const Eigen::Matrix3d &R) 
{
  Eigen::Vector4d q = {0,0,0,0};
  double tr = R.trace();
  double S;
  if (tr > 0.0) {
    S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    q(0) = 0.25 * S;
    q(1) = (R(2, 1) - R(1, 2)) / S;
    q(2) = (R(0, 2) - R(2, 0)) / S;
    q(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    q(0) = (R(2, 1) - R(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (R(0, 1) + R(1, 0)) / S;
    q(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    q(0) = (R(0, 2) - R(2, 0)) / S;
    q(1) = (R(0, 1) + R(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    q(0) = (R(1, 0) - R(0, 1)) / S;
    q(1) = (R(0, 2) + R(2, 0)) / S;
    q(2) = (R(1, 2) + R(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}


Eigen::Matrix3d latencyCorrection::quat2rot(const Eigen::Vector4d &q) 
{
  Eigen::Matrix3d R;
  R << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return R;
}

void latencyCorrection::localposeCb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    //cout<<"lpf: "<<lpf<<endl;
    Eigen::Vector4d q;
    q(0) = msg->pose.orientation.w;
    q(1) = msg->pose.orientation.x;
    q(2) = msg->pose.orientation.y;
    q(3) = msg->pose.orientation.z;
    Eigen::Matrix3d mat = quat2rot(q);
    double time = msg->header.stamp.toSec();

    lp.first = time;
    lp.second = Eigen::Matrix4d::Identity();
    lp.second.block(0,0,3,3) = mat;
    lp.second(0,3) = msg->pose.position.x;
    lp.second(1,3) = msg->pose.position.y;
    lp.second(2,3) = msg->pose.position.z;
    localpose.push_back(lp);

    // an extra vector just for storing the times of localpose    
    localpose_times.push_back(time);
    
    // remove the element when the length is more than 1000
    if (localpose.size() > 1000)
    {
      localpose.erase(localpose.begin());
      localpose_times.erase(localpose_times.begin());
    }


    //cout << "size of vision pose:" << visionpose.size() <<endl;
    Eigen::Matrix4d T_local_curr, T_local_delt, T_cloud_delt, T_corrected;
    if (localpose.size() > 100)
    {
      
      //auto it = std::lower_bound(localpose_times.begin(), localpose_times.end(), cloudpose.first);
      //int j = std::distance(localpose_times.begin(), it);
      //if (j == localpose_times.size()) {j = j-1;}

      int j;
      for (int i = localpose_times.size()-1; i >= 0; --i)
      {
        if (localpose_times[i] < cloudpose.first)
        {
          j = i;
          break;
        }
        else
        {
          j = localpose_times.size()-1;
        }
      
      }

      T_local_curr = localpose.back().second;
      T_local_delt = localpose[j].second;
      T_cloud_delt = cloudpose.second;
      T_corrected = T_cloud_delt * T_local_delt.inverse() * T_local_curr;

      Eigen::Matrix3d R = T_corrected.block(0,0,3,3);
      Eigen::Vector4d qq = rot2quat(R);

      Eigen::Vector3d V = {0, 0, 0};
      X_corrected.first = X_corrected.second; 
      X_corrected.second << T_corrected(0,3), T_corrected(1,3), T_corrected(2,3);
      V = (X_corrected.second - X_corrected.first) * lpf;


      nav_msgs::Odometry corrected_odom; 
      corrected_odom.header.stamp = msg->header.stamp;
      corrected_odom.header.frame_id = "map";
      
      corrected_odom.pose.pose.position.x = T_corrected(0, 3);
      corrected_odom.pose.pose.position.y = T_corrected(1, 3);
      corrected_odom.pose.pose.position.z = T_corrected(2, 3);

      corrected_odom.pose.pose.orientation.x = qq(1);
      corrected_odom.pose.pose.orientation.y = qq(2);
      corrected_odom.pose.pose.orientation.z = qq(3);
      corrected_odom.pose.pose.orientation.w = qq(0);

      corrected_odom.twist.twist.linear.x = V(0);
      corrected_odom.twist.twist.linear.y = V(1);
      corrected_odom.twist.twist.linear.z = V(2);

      odom_pub.publish(corrected_odom);

      geometry_msgs::PoseStamped mv_pose;
      mv_pose.header.stamp = msg->header.stamp;

      mv_pose.pose.position.x = T_corrected(0, 3); 
      mv_pose.pose.position.y = T_corrected(1, 3); 
      mv_pose.pose.position.z = T_corrected(2, 3);

      mv_pose.pose.orientation.x = qq(1);
      mv_pose.pose.orientation.y = qq(2);
      mv_pose.pose.orientation.z = qq(3);
      mv_pose.pose.orientation.w = qq(0);

      mavros_pub.publish(mv_pose);
    



    }

}

void latencyCorrection::cloudposeCb(const nav_msgs::Odometry::ConstPtr& msg) 
{
    Eigen::Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    Eigen::Matrix3d mat = q.toRotationMatrix();

    double time = msg->header.stamp.toSec();
    cloudpose.first = time;
    cloudpose.second = Eigen::Matrix4d::Identity();
    cloudpose.second.block(0,0,3,3) = mat;
    cloudpose.second(0,3) = msg->pose.pose.position.x;
    cloudpose.second(1,3) = msg->pose.pose.position.y;
    cloudpose.second(2,3) = msg->pose.pose.position.z;

}


int main(int argc, char **argv)
{

    // initialize latency correction node
    ros::init(argc, argv, "latency_correction");
    ros::NodeHandle nh;
    latencyCorrection lc;

    std::string cloudpose_topic, localpose_topic, odom_topic, mavros_topic;
    //double latency, cpf, T_cpf;
    nh.param<std::string>("input_odometry_topic", cloudpose_topic, "/udrone1/final_odometry");
    nh.param<std::string>("localpose_topic", localpose_topic, "/udrone1/mavros/local_position/pose");
    nh.param<std::string>("odom_topic", odom_topic, "/udrone1/compansated_odometry");
    nh.param<std::string>("output_mavros_topic", mavros_topic, "/udrone1/mavros/vision_pose/pose");
    //nh.param<double>("latency", latency, 0.0);
    //nh.param<double>("cloudpose_frequency", cpf, 20.0);
    nh.param<double>("localpose_frequency", lpf, 30.0);
    //T_cpf = 1.0/cpf;
    //cout<< "cloudpose_topic: "<< cloudpose_topic.c_str() << endl; 
    cout<< "localpose_topic: "<< localpose_topic.c_str() << endl;
    cout<< "odom_topic: "<< odom_topic.c_str() << endl;
    //cout<< "latency: "<< latency << endl;
    //cout<< "cloudpose_frequency: "<< cpf << endl;
    cout<< "localpose_frequency: "<< lpf << endl;

    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic.c_str(), 1);
    mavros_pub = nh.advertise<geometry_msgs::PoseStamped>(mavros_topic.c_str(), 1);
    ros::Subscriber sub1 = nh.subscribe(localpose_topic.c_str(), 1, &latencyCorrection::localposeCb, &lc);
    ros::Subscriber sub2 = nh.subscribe(cloudpose_topic.c_str(), 1, &latencyCorrection::cloudposeCb, &lc);

    //message_filters::Subscriber<nav_msgs::Odometry> sub3(nh, cloudpose_topic.c_str(), 80);
    //message_filters::TimeSequencer<nav_msgs::Odometry> seq(sub3, ros::Duration(latency), ros::Duration(T_cpf), 80);
    //seq.registerCallback(&latencyCorrection::cloudposeCb, &lc);

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}

