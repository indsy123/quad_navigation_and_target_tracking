#include<controller.h>

using namespace std;
using namespace Eigen;
//using std::ofstream;
//std::ofstream outdata;

void controller::getParamters()
{
  // get all the parameters from launch file

  nh_.param<std::string>("pose_topic", pose_topic, "/iris/ground_truth/odometry");    
  nh_.param<std::string>("trajectory_topic", trajectory_topic, "/quadrotor/desired_trajectory");
  nh_.param<double>("mass", m, 1.6);    
  nh_.param<double>("normalized_thrust_constant", Tc, 35.0);
  nh_.param<double>("publish_frequency", Hz, 30.0);
  nh_.param<double>("rate_gain", f, 0.2);
  nh_.param<std::vector <double>>("final_goal", final_goal, {0, 0, 2});  
  nh_.param<std::vector <double>>("kx", kx, {5.0, 5.0, 4.0});  
  nh_.param<std::vector <double>>("kv", kv, {3.0, 3.0, 2.0});  
}

void controller::initializeSubscribers()
{
  start_time = ros::Time::now();
  ROS_INFO("Initializing Subscribers");
  state_sub =  nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &controller::mavros_state_cb, this);
  arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


  odom_sub = nh_.subscribe(pose_topic.c_str(), 1, &controller::odom_cb, this);  
  //trajectory_sub = nh_.subscribe(trajectory_topic.c_str(), 1, &controller::trajectory_cb, this);  
}

void controller::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  bool latch;
  cmd_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
}


void controller::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_mavros_state = *msg;
}

Eigen::Vector4d controller::rot2quat(const Eigen::Matrix3d &R) 
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


Eigen::Vector4d controller::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) 
{
  Eigen::Vector4d quat = {0,0,0,0};
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d controller::quat2rot(const Eigen::Vector4d &q) 
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

void controller::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
  
  geometry_msgs::PoseStamped mv_pose;
  mv_pose.header.stamp = msg->header.stamp; 
  mv_pose.pose = msg->pose.pose;
  pose_pub.publish(mv_pose);

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;

  if( current_mavros_state.mode != "OFFBOARD" && (ros::Time::now() - start_time > ros::Duration(5.0)))
  {
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
      ROS_INFO("Offboard enabled");
    }
  } 
  else 
  {
    if( !current_mavros_state.armed && (ros::Time::now() - start_time > ros::Duration(5.0)))
    {
      if( arming_client.call(arm_cmd) && arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed");
      }
    }
  }
    
  //if (current_mavros_state.mode == "OFFBOARD" && current_mavros_state.armed)
  //{
    // current state
  Eigen::Vector3d Xc = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  Eigen::Vector3d Vc = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  Eigen::Vector4d qc;
  qc(0) = msg->pose.pose.orientation.w;
  qc(1) = msg->pose.pose.orientation.x;
  qc(2) = msg->pose.pose.orientation.y;
  qc(3) = msg->pose.pose.orientation.z;
  Eigen::Matrix3d Rc;
  Rc = quat2rot(qc);
  
  

  //desired state, hard-coded for quick check, eventually be taken from the trajectoy_msg
  // it wont be final goal but a point on the trajectory
  Eigen::Vector3d Xd, Vd, ad, kp, kd, ex, ev, _b1d, b1d, _b3d, b3d, b2d, eR;
  Eigen::Matrix3d Rd, _eR;
  Eigen::Vector4d qd;
  double normalized_thrust;

  Xd << final_goal[0], final_goal[1], final_goal[2];
  Vd = {0, 0, 0};
  ad = {0, 0, 0};

  //get gains, this should be changed, see why direct Vector3f is not coming from launch
  // or check if std::vector can be multiplied
  kp << kx[0], kx[1], kx[2];
  kd << kv[0], kv[1], kv[2];

  ex = Xc - Xd;
  ev = Vc - Vd;

  _b1d = {1, 0, 0}; // hard coded for now

  _b3d = -(kp.asDiagonal() * ex + kd.asDiagonal() * ev)/ m + ad + g;
  b3d = _b3d.normalized();
  b2d = (b3d.cross(_b1d)).normalized();
  b1d = (b2d.cross(b3d)).normalized();

  Rd << b1d(0), b2d(0), b3d(0), b1d(1), b2d(1), b3d(1), b1d(2), b2d(2), b3d(2);
  qd = rot2quat(Rd);

  _eR = 0.5 * (Rd.transpose() * Rc - Rc.transpose() * Rd); 

  //cout << _eR(0,0) << ", " << _eR(0,1) << ", " << _eR(0,2) << endl;
  //cout << _eR(1,0) << ", " << _eR(1,1) << ", " << _eR(1,2) << endl;
  //cout << _eR(2,0) << ", " << _eR(2,1) << ", " << _eR(2,2) << endl;

  eR << _eR(7), _eR(2), _eR(3);

  //cout << eR(0) << ", " << eR(1) << ", " << eR(2) << endl;


  Eigen::Vector4d qe, q_inv, inverse;
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * qc;  
  qe = quatMultiplication(q_inv, qd);


  normalized_thrust = m * _b3d.dot(Rc.col(2)) / Tc; 
  mavros_msgs::AttitudeTarget cmd;
  cmd.header.stamp = msg->header.stamp;
  cmd.type_mask = 128;
  cmd.body_rate.x = (2.0/f) * eR(0);
  cmd.body_rate.y = (2.0/f) * eR(1);
  cmd.body_rate.z = (2.0/f) * eR(2);
  cmd.thrust = std::max(0.0, std::min(1.0, normalized_thrust ));

  //cmd.body_rate.x = (2.0 / f) * std::copysign(1.0, qe(0)) * qe(1);
  //cmd.body_rate.y = (2.0 / f) * std::copysign(1.0, qe(0)) * qe(2);
  //cmd.body_rate.z = (2.0 / f) * std::copysign(1.0, qe(0)) * qe(3);
  //cmd.thrust = std::max(0.0, std::min(1.0, m * _b3d.dot(Rc.col(2))/Tc ));  // Calculate thrust

  cout << "rate_x: " <<cmd.body_rate.x << ", rate_y: " <<cmd.body_rate.y<< ", rate_z: " <<cmd.body_rate.z<<", thrust: "<< cmd.thrust<<endl;
  cmd_pub.publish(cmd);
  global_counter += 1;
  //}

}

// have to modify it 
void controller::trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) 
{

    for (int i = 0; i < msg->points.size(); i++)
    {
      //int j = global_counter *  msg->points.size() + i;
      double tt = msg->points[i].time_from_start.toSec();
      //cout << "ttttt:" << tt<< endl;
      traj_global_time.push_back(tt);
      xdes.push_back(msg->points[i].transforms[0].translation.x);
      ydes.push_back(msg->points[i].transforms[0].translation.y);
      vxdes.push_back(msg->points[i].velocities[0].linear.x);
      vydes.push_back(msg->points[i].velocities[0].linear.y);
      double thdes = tf2::getYaw(msg->points[i].transforms[0].rotation);
      thetades.push_back(thdes);
      wdes.push_back(msg->points[i].velocities[0].angular.z);

      if (traj_global_time.size() >= 5000)
      {
        traj_global_time.erase(traj_global_time.begin());
        xdes.erase(xdes.begin());
        ydes.erase(ydes.begin());
        vxdes.erase(vxdes.begin());
        vydes.erase(vydes.begin());
        thetades.erase(thetades.begin());
        wdes.erase(wdes.begin());
      }

    }
    
    TrajectoryPublished = true;

}

