#include <LocalPlanner.h>
#include <TrajectoryGenerationOptimalControl.h>
#include <RayGeneration.h>
#include<chrono>
#include <sstream>
#include <fstream>
#include <iostream>


using namespace std;
Eigen::Matrix4f lidar_pose;


void localplanner::getParamters()
{
  // get all the parameters from launch file
  nh_.param<std::string>("pose_topic", pose_topic, "/odometry_topic");    
  nh_.param<std::string>("pointcloud_topic", pointcloud_topic, "/pointcloud_topic");
  nh_.param<double>("odometry_frequency", odometry_frequency, 20.0);    
  nh_.param<double>("lidar_frequency", lidar_frequency, 10.0);
  nh_.param<double>("average_speed", av_speed, 1.0);    
  nh_.param<double>("max_sensor_range", SensorRangeMax, 5.0);
  nh_.param<double>("min_sensor_range", SensorRangeMin, 2.0);
  nh_.param<double>("sensor_fov", SensorFOV, 120.0);
  nh_.param<vector<double>>("final_goal", final_goal, {0,0});
  ch_time = 1.1 * (1.0/lidar_frequency);
  delt = 1.0/odometry_frequency;

  //print parameters to crosscheck
  cout <<"pose_topic: "<< pose_topic <<endl;
  cout <<"pointcloud_topic: "<< pointcloud_topic <<endl;
  cout <<"odometry_frequency: "<< odometry_frequency <<endl;
  cout <<"lidar_frequency: "<< lidar_frequency <<endl;
  cout <<"av_speed: "<< av_speed <<endl;
  cout <<"SensorRangeMax: "<< SensorRangeMax <<endl;
  cout <<"SensorRangeMin: "<< SensorRangeMin <<endl;
  cout <<"final_goal: "<< final_goal[0] << ", "<< final_goal[1] <<endl;
}

void localplanner::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  odom_sub = nh_.subscribe(pose_topic.c_str(), 1, &localplanner::odometry_callback, this);  
  pc_sub = nh_.subscribe(pointcloud_topic.c_str(), 1, &localplanner::pointcloud_callback, this);  
}

void localplanner::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  bool latch;
  traj_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/jackal/desired_trajectory", 1, latch=true);
  best_traj_pub = nh_.advertise<sensor_msgs::PointCloud2>("/best_trajectory", 1, latch=true);
  ensamble_pub = nh_.advertise<sensor_msgs::PointCloud2>("/ensamble", 1, latch=true);
}


void localplanner::PublishTrajectory(const sensor_msgs::PointCloud2ConstPtr& cloud, VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t, VectorXd& XXddot, VectorXd& YYddot)
{
    trajectory_msgs::MultiDOFJointTrajectory traj;
    traj.points.resize(t.size());
    traj.joint_names.resize(1);
    traj.joint_names[0] ="jackal_base_link";
    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    double start_time = ros::Time::now().toSec();

    for (int i = 0; i < t.size(); i++)
    {
      outdata.open("generated_trajectory.txt", std::ios::out|std::ios::binary);  
      outdata << t(i) << "," << XX(i) << "," << YY(i) << "," << XXdot(i) << "," << YYdot(i) << "," << theta(i) << "," << endl;
      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      point.transforms.resize(1);
      point.velocities.resize(1);
      point.accelerations.resize(1);

      double _time = start_time + t(i);

      point.transforms[0].translation.x = XX(i);
      point.transforms[0].translation.y = YY(i);

      tf2::Quaternion q;
      q.setRPY(0, 0, theta(i));
      point.transforms[0].rotation.x = q[0];
      point.transforms[0].rotation.y = q[1];
      point.transforms[0].rotation.z = q[2];
      point.transforms[0].rotation.w = q[3];

      point.velocities[0].linear.x = XXdot(i);
      point.velocities[0].linear.y = YYdot(i);
      point.velocities[0].angular.z = thetadot(i);

      point.accelerations[0].linear.x = XXddot(i);
      point.accelerations[0].linear.y = YYddot(i);

      point.time_from_start  = ros::Duration(_time);
      traj.points[i] = point;      
      
    }
    traj_pub.publish(traj);
    ROS_INFO_STREAM("Publishing polynomial trajectory.");


}

void localplanner::publishTransform(const nav_msgs::Odometry::ConstPtr& msg)
{
  static tf::TransformBroadcaster br1, br2;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  tf::Quaternion q;
  q[0]  = msg->pose.pose.orientation.x;
  q[1]  = msg->pose.pose.orientation.y;
  q[2]  = msg->pose.pose.orientation.z;
  q[3]  = msg->pose.pose.orientation.w;
  transform.setRotation(q);
  br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));
  br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/velodyne"));
}

void localplanner::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (pose_msg_counter == 0)
  {
    // ch_point set to intial position of the robot
    ch_point[0] = msg->pose.pose.position.x;
    ch_point[1] = 0.1;//msg->twist.twist.linear.x; //started with a small initial velocity always
    ch_point[2] = 0.0; // sensor doesnt give acceleration, so a small initial value
    ch_point[3] = msg->pose.pose.position.y;         
    ch_point[4] = 0.0;// no need to give any velocity in y direction 
    ch_point[5] = 0.0; //initilized with 0 accelration 
    ch_point[6] = tf2::getYaw(msg->pose.pose.orientation);
    ch_point[7] = msg->twist.twist.angular.z;
  }

  curr_pose[0] = msg->pose.pose.position.x;
  curr_pose[1] = msg->twist.twist.linear.x;
  curr_pose[2] = 0.0;
  curr_pose[3] = msg->pose.pose.position.y;      
  curr_pose[4] = msg->twist.twist.linear.y;
  curr_pose[5] = 0.0;
  curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
  curr_pose[7] = msg->twist.twist.angular.z;

  publishTransform(msg);

  Eigen::Quaternionf q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3f mat = q.toRotationMatrix();
  lidar_pose = Eigen::Matrix4f::Identity();
  lidar_pose.block(0,0,3,3) = mat;
  lidar_pose(0,3) = msg->pose.pose.position.x;
  lidar_pose(1,3) = msg->pose.pose.position.y;
  lidar_pose(2,3) = msg->pose.pose.position.z;

  pose_msg_counter += 1;
}

void localplanner::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud) 
{

    clock_t start, end;
    start = clock(); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);

    
    //pcl::fromROSMsg(cloud_out, *cloud_in);
    pcl::fromROSMsg(*cloud, *cloud_in);

    pcl::transformPointCloud (*cloud_in, *cloud_trans, lidar_pose);

    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud (cloud_trans);
    fc.setVerticalFOV (SensorFOV);
    fc.setHorizontalFOV (SensorFOV);
    fc.setNearPlaneDistance (0.0);
    fc.setFarPlaneDistance (SensorRangeMax);

    // .. read or input the camera pose from a registration algorithm.
    fc.setCameraPose (lidar_pose);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr truncated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fc.filter (*truncated_cloud);

    
    RayGeneration rg;
    vector< vector<double> > points;
    vector< vector<double> > availablepoints;

    vector<double> local_goal(2);

    pcl::PointCloud<pcl::PointXYZ> ensamble_cloud;

    rg.GenerateEnsamble(curr_pose, SensorRangeMax, SensorRangeMin, SensorFOV, points);
    rg.CheckCollosion_GetCost(truncated_cloud, final_goal, av_speed, ch_point, points, availablepoints, ensamble_cloud);    
    rg.GenerateLocalGoal(availablepoints, local_goal);

    if (ContinueReplanning)
    {
      VectorXd px(6);
      VectorXd py(6);
      double T;
  
      OptimalControlTrajectory Traj;
      Traj.PolynomialCoeff(ch_point, local_goal, av_speed, px, py, T);

      
      Distance2Goal = sqrt(pow(final_goal[0]-curr_pose[0], 2) +  pow(final_goal[1]-curr_pose[3], 2));
 
      if (Distance2Goal < SensorRangeMax){ FinalGoalinFOV = true;}

      if (!FinalGoalinFOV)
      {
        intervals = int(ch_time / delt)+1;
      }
      else
      { 
        ch_time = T;
        intervals = int(ch_time / delt)+1;
        ContinueReplanning = false;
      }
      cout<<ch_time<<","<<T<<", "<<delt<<endl;
      VectorXd t(intervals);
      VectorXd XX(intervals);
      VectorXd YY(intervals);
      VectorXd theta(intervals);
      VectorXd XXdot(intervals);
      VectorXd YYdot(intervals);
      VectorXd XXddot(intervals);
      VectorXd YYddot(intervals);
      VectorXd thetadot(intervals);

      for (int i = 0; i < intervals; i++)
      { 
        if (i == 0){ t(i) = 0;}
        else{t(i) = t(i-1) + delt;}       
      }
      
      Traj.getControlHorizonPoint(px, py, ch_time, ch_point);
      Traj.PolynomialTrajectory(px, py, T, t, XX, YY, theta, XXdot, YYdot, thetadot, XXddot, YYddot);

     
      for(int i = 0; i < XX.size(); i++)
      {
        pcl::PointXYZ pt;
        pt.x = XX(i);
        pt.y = YY(i);
        pt.z = 0.0;
        traj_cloud.points.push_back(pt);
      }
      sensor_msgs::PointCloud2 pc_traj;
      pcl::toROSMsg(traj_cloud, pc_traj);
      pc_traj.header.frame_id = "map";
      pc_traj.header.stamp = ros::Time::now();
      best_traj_pub.publish(pc_traj);


      sensor_msgs::PointCloud2 pc_ensamble;
      pcl::toROSMsg(ensamble_cloud, pc_ensamble);
      pc_ensamble.header.frame_id = "map";
      pc_ensamble.header.stamp = ros::Time::now();
      ensamble_pub.publish(pc_ensamble);

    
      PublishTrajectory(cloud, XX, YY, theta, XXdot, YYdot, thetadot, t, XXddot, YYddot);

    }  
    else
    {
      cout << " The final goal is within the FOV, static trajectory is being tracked" << endl; 
    }
    


    end = clock(); 
    double time_taken2 = double(end - start) / double(CLOCKS_PER_SEC); 
    cout << "Time taken in publishing is : " << fixed << time_taken2 << setprecision(7) << "sec" << endl; 

}

