#include <vio2cog_propagte_and_transform.h>

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
  nh_.param<double>("input_pose_frequency", pose_frequency, 30.0);  
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
  dt_imu = 1.0/imu_frequency;


}

void vio2cog::initializeSubscribers()
{
  ROS_INFO("Initializing Planner Subscribers");
  odom_sub = nh_.subscribe(input_odom.c_str(), 100, &vio2cog::odometry_callback, this);  
  //imu_sub = nh_.subscribe(input_imu.c_str(), 1, &vio2cog::imu_callback, this);  
}

void vio2cog::initializePublishers()
{
  ROS_INFO("Initializing Planner Publishers");
  bool latch;
  odom_pub = nh_.advertise<nav_msgs::Odometry>(output_odom.c_str(), 100);
  mavros_pub = nh_.advertise<geometry_msgs::PoseStamped>(mavros_topic.c_str(), 100);
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

  

  /*if (odom_counter = 0)
  {
    RGtoI_dot = Eigen::Matrix3d::Identity();
    RGtoI_previous = RGtoI;
  }
  else
  {
    RGtoI_dot = (RGtoI - RGtoI_previous)/dt_odom;
    RGtoI_previous = RGtoI;
  }*/

  RGtoI_dot = RGtoI * omega_hat;

  RMtoCoG = RMtoG * RGtoI * RItoCoG;

	pCoGinM = pGinM + RMtoG * (pIinG + (RGtoI * pCoGinI));
  vCoGinM = vGinM + RMtoG * (vIinG + (RGtoI_dot * pCoGinI));

  Eigen::Vector4d q_cg;
  q_cg  = rot2quat(RMtoCoG);


  nav_msgs::Odometry msg1;
  msg1.header.stamp = msg->header.stamp;
  msg1.pose.pose.position.x = pCoGinM(0); 
  msg1.pose.pose.position.y = pCoGinM(1); 
  msg1.pose.pose.position.z = pCoGinM(2);

  msg1.pose.pose.orientation.x = q_cg(1); 
  msg1.pose.pose.orientation.y = q_cg(2);
  msg1.pose.pose.orientation.z = q_cg(3); 
  msg1.pose.pose.orientation.w = q_cg(0);

  msg1.twist.twist.linear.x = vCoGinM(0); 
  msg1.twist.twist.linear.y = vCoGinM(1); 
  msg1.twist.twist.linear.z = vCoGinM(2);
  odom_pub.publish(msg1);

  odom_counter += 1;
  fast_propagate = true;

  geometry_msgs::PoseStamped msg2;
  msg2.header.stamp = msg->header.stamp;

  msg2.pose.position.x = pCoGinM(0); msg2.pose.position.y = pCoGinM(1); msg2.pose.position.z = pCoGinM(2);

  msg2.pose.orientation.x = q_cg(1); msg2.pose.orientation.y = q_cg(2);
  msg2.pose.orientation.z = q_cg(3); msg2.pose.orientation.w = q_cg(0);
  mavros_pub.publish(msg2);
}

void vio2cog::imu_callback(const sensor_msgs::ImuPtr& msg)
{
  if (fast_propagate)
  {
    Eigen::Vector3d w_measured, a_measured;
    Eigen::Matrix3d w_hat; 

    w_measured << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    a_measured << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // initialize G frame, i.e. get the value of g
    if (imu_counter == 0)
    {
      g = a_measured;
    }
    else if (imu_counter < 50)
    {
      g_sum = g_sum + a_measured;
      g = g_sum/(imu_counter + 1); 
    }
    //cout << "g:"<<g(0)<<","<<g(1)<<","<<g(2)<<endl;
    //cout <<"imu:w:"<<w_measured(0)<<","<<w_measured(1)<<","<<w_measured(2)<<endl;
    //cout <<"imu:a:"<<a_measured(0)<<","<<a_measured(1)<<","<<a_measured(2)<<endl;

    w_hat << 0, -w_measured(2), w_measured(1), w_measured(2), 0, -w_measured(0), -w_measured(1), w_measured(0), 0;

    //Eigen::Vectpr3d X, V;
    //Eigen::Matrix3d R;

    pCoGinM = pCoGinM + vCoGinM * dt_imu;
    
    RMtoCoG = RMtoCoG * (Eigen::Matrix3d::Identity() + w_hat*dt_imu);
    vCoGinM = vCoGinM + dt_imu * (RMtoCoG * (a_measured - g));

    //cout <<"imu:pCoGinM:"<<pCoGinM(0)<<","<<pCoGinM(1)<<","<<pCoGinM(2)<<endl;
    //cout <<"imu:vCoGinM:"<<vCoGinM(0)<<","<<vCoGinM(1)<<","<<vCoGinM(2)<<endl;

    //self.X = self.X + self.V * self.dt 
    //self.R = np.dot(self.R, (np.identity(3) + w_hat*self.dt))
    //se/lf.V = self.V + self.dt * (np.dot(self.R, a[np.newaxis].T)-self.g)

    Eigen::Vector4d q_cg;
    q_cg  = rot2quat(RMtoCoG);

    nav_msgs::Odometry msg1;
    msg1.header.stamp = msg->header.stamp;
    msg1.pose.pose.position.x = pCoGinM(0); msg1.pose.pose.position.y = pCoGinM(1); msg1.pose.pose.position.z = pCoGinM(2);

    msg1.pose.pose.orientation.x = q_cg(1); msg1.pose.pose.orientation.y = q_cg(2);
	  msg1.pose.pose.orientation.z = q_cg(3); msg1.pose.pose.orientation.w = q_cg(0);

    msg1.twist.twist.linear.x = vCoGinM(0); msg1.twist.twist.linear.y = vCoGinM(1); msg1.twist.twist.linear.z = vCoGinM(2);
    odom_pub.publish(msg1);

    geometry_msgs::PoseStamped msg2;
    msg2.header.stamp = msg->header.stamp;

    msg2.pose.position.x = pCoGinM(0); msg2.pose.position.y = pCoGinM(1); msg2.pose.position.z = pCoGinM(2);

    msg2.pose.orientation.x = q_cg(1); msg2.pose.orientation.y = q_cg(2);
	  msg2.pose.orientation.z = q_cg(3); msg2.pose.orientation.w = q_cg(0);
    mavros_pub.publish(msg2);

    imu_counter += 1; 
  } 
}

    

