#include <vio2cog_transform.h>

using namespace std;
Eigen::Matrix4f lidar_pose;
//std::ofstream outdata;
//outdata.open("generated_trajectory.txt", std::ios::out|std::ios::binary);


void vio2cog::getParameters()
{
  
  ROS_INFO("getting all the parameters from launch file");
  // planner parameters
  nh_.param<std::string>("input_odometry_topic", input_odom, "/odometry_topic");    
  nh_.param<double>("input_odometry_frequency", odom_frequency, 100.0);  
  nh_.param<std::string>("input_pose_topic", input_pose, "/pose_topic");    
  //nh_.param<double>("input_pose_frequency", pose_frequency, 30.0);  
  nh_.param<std::string>("output_odometry_topic", output_odom, "/output_topic");
  nh_.param<std::string>("output_mavros_topic", mavros_topic, "/output_mavros");

  nh_.param<std::vector <double>>("pGinM", _pGinM, {0,0,0});
  nh_.param<std::vector <double>>("vGinM", _vGinM, {0,0,0});  
  nh_.param<std::vector <double>>("pCoGinI", _pCoGinI, {0,0,0});
  nh_.param<std::vector <double>>("RMtoG", _RMtoG, {0, 0, 0, 0, 0, 0, 0, 0, 0});
  nh_.param<std::vector <double>>("RItoCoG", _RItoCoG, {0, 0, 0, 0, 0, 0, 0, 0, 0});


  pGinM << _pGinM[0], _pGinM[1], _pGinM[2];
  vGinM << _vGinM[0], _vGinM[1], _vGinM[2];
  pCoGinI <<_pCoGinI[0], _pCoGinI[1], _pCoGinI[2];

  RMtoG << _RMtoG[0], _RMtoG[1], _RMtoG[2], 
           _RMtoG[3], _RMtoG[4], _RMtoG[5], 
           _RMtoG[6], _RMtoG[7], _RMtoG[8];

  RItoCoG << _RItoCoG[0], _RItoCoG[1], _RItoCoG[2], 
             _RItoCoG[3], _RItoCoG[4], _RItoCoG[5],
             _RItoCoG[6], _RItoCoG[7], _RItoCoG[8];
  
  cout <<"pGinM:"<<pGinM(0)<<","<<pGinM(1)<<","<<pGinM(2)<<endl;
  cout <<"vGinM:"<<vGinM(0)<<","<<vGinM(1)<<","<<vGinM(2)<<endl;
  cout <<"pCoGinI:"<<pCoGinI(0)<<","<<pCoGinI(1)<<","<<pCoGinI(2)<<endl;

  cout << "RMtoG:"<< RMtoG(0)<<","<< RMtoG(3)<<","<< RMtoG(6)<<","
                  << RMtoG(1)<<","<< RMtoG(4)<<","<< RMtoG(7)<<","
                  << RMtoG(2)<<","<< RMtoG(5)<<","<< RMtoG(8)<<endl;

  cout << "RItoCoG:"<< RItoCoG(0)<<","<< RItoCoG(3)<<","<< RItoCoG(6)<<","
                    << RItoCoG(1)<<","<< RItoCoG(4)<<","<< RItoCoG(7)<<","
                    << RItoCoG(2)<<","<< RItoCoG(5)<<","<< RItoCoG(8)<<endl;

  dt_odom = 1.0/odom_frequency;
  //dt_imu = 1.0/imu_frequency;


}

void vio2cog::initializeSubscribers()
{
  ROS_INFO("Initializing Planner Subscribers");
  odom_sub = nh_.subscribe(input_odom.c_str(), 200, &vio2cog::odometry_callback, this, ros::TransportHints().tcpNoDelay(true));  
  pose_sub = nh_.subscribe(input_pose.c_str(), 10, &vio2cog::pose_callback, this, ros::TransportHints().tcpNoDelay(true));  
}

void vio2cog::initializePublishers()
{
  ROS_INFO("Initializing Planner Publishers");
  bool latch;
  odom_pub = nh_.advertise<nav_msgs::Odometry>(output_odom.c_str(), 20);
  mavros_pub = nh_.advertise<geometry_msgs::PoseStamped>(mavros_topic.c_str(), 1);
  mavros_odom_pub = nh_.advertise<nav_msgs::Odometry>("/udrone1/mavros/odometry/out", 20);
}


Eigen::Vector4d vio2cog::rot2quat(const Eigen::Matrix3d &R) 
{
  // not needed for now
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


void vio2cog::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ros::Rate rate(30.0);
  Eigen::Vector3d pIinG, vIinG, omega;
  Eigen::Matrix3d omega_hat;

  pIinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  vIinG << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  omega << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
  omega_hat << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0;

  Eigen::Quaterniond q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3d RGtoI = q.toRotationMatrix();

  

  if (odom_counter = 0)
  {
    RGtoI_dot = Eigen::Matrix3d::Identity();
    RGtoI_previous = RGtoI;
  }
  else
  {
    RGtoI_dot = (RGtoI - RGtoI_previous)/dt_odom;
    RGtoI_previous = RGtoI;
  }

  //RGtoI_dot = RGtoI * omega_hat;

  RMtoCoG = RMtoG * RGtoI * RItoCoG;

  pCoGinM = pGinM + RMtoG * (pIinG + (RGtoI * pCoGinI));
  vCoGinM = vGinM + RMtoG * (vIinG + (RGtoI_dot * pCoGinI));
 
  //std::vector<double> v;
  //std::vector<double> _vCoGinM;
  //v = {vCoGinM(0), vCoGinM(1), vCoGinM(2)}; 
  //sum_vCoGinM.push_back(v);

  vx.push_back(vCoGinM(0)); vy.push_back(vCoGinM(1)); vz.push_back(vCoGinM(2));
  double _vx, _vy,_vz;

  if (vx.size() >= Nsamples)
  {
    _vx = std::accumulate(vx.begin(), vx.end(), 0.0)/Nsamples;
    _vy = std::accumulate(vy.begin(), vy.end(), 0.0)/Nsamples;
    _vz = std::accumulate(vz.begin(), vz.end(), 0.0)/Nsamples;
    vx.erase(vx.begin()); vy.erase(vy.begin()); vz.erase(vz.begin());
  }  
    
  outdata <<vIinG(0)<<","<<vCoGinM(0)<<","<<_vx<<","<<vIinG(1)<<","<<vCoGinM(1)<<","<<_vy<<","<<vIinG(2)<<","<<vCoGinM(2)<<","<<_vz<<endl;

   

  Eigen::Vector4d q_cg;
  q_cg  = rot2quat(RMtoCoG);


  nav_msgs::Odometry msg1;
  msg1.header.stamp = ros::Time::now();//msg->header.stamp;
  msg1.pose.pose.position.x = pCoGinM(0); 
  msg1.pose.pose.position.y = pCoGinM(1); 
  msg1.pose.pose.position.z = pCoGinM(2);

  msg1.pose.pose.orientation.x = q_cg(1); 
  msg1.pose.pose.orientation.y = q_cg(2);
  msg1.pose.pose.orientation.z = q_cg(3); 
  msg1.pose.pose.orientation.w = q_cg(0);

  msg1.twist.twist.linear.x = _vx; 
  msg1.twist.twist.linear.y = _vy; 
  msg1.twist.twist.linear.z = _vz;

  odom_pub.publish(msg1);
  //mavros_odom_pub.publish(msg1);
  odom_counter += 1; 
  //rate.sleep();

}

void vio2cog::pose_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{

  Eigen::Vector3d pIinG, vIinG, omega;
  //Eigen::Matrix3d omega_hat;

  pIinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;


  Eigen::Quaterniond q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3d RGtoI = q.toRotationMatrix();


  //RGtoI_dot = RGtoI * omega_hat;

  RMtoCoG = RMtoG * RGtoI * RItoCoG;

  pCoGinM = pGinM + RMtoG * (pIinG + (RGtoI * pCoGinI));
  //vCoGinM = vGinM + RMtoG * (vIinG + (RGtoI_dot * pCoGinI));

  Eigen::Vector4d q_cg;
  q_cg  = rot2quat(RMtoCoG);

  geometry_msgs::PoseStamped msg2;
  msg2.header.stamp = ros::Time::now();//msg->header.stamp;

  msg2.pose.position.x = pCoGinM(0); 
  msg2.pose.position.y = pCoGinM(1); 
  msg2.pose.position.z = pCoGinM(2);

  msg2.pose.orientation.x = q_cg(1); 
  msg2.pose.orientation.y = q_cg(2);
  msg2.pose.orientation.z = q_cg(3); 
  msg2.pose.orientation.w = q_cg(0);

  mavros_pub.publish(msg2);


}

    

