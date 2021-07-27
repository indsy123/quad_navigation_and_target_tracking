#include <planner.h>


using namespace std;
Eigen::Matrix4f lidar_pose;
std::ofstream outdata;
//outdata.open("generated_trajectory.txt", std::ios::out|std::ios::binary);


void planner::getParameters()
{
  
  ROS_INFO("getting all the parameters from launch file");
  // planner parameters
  nh_.param<std::string>("mode", mode, "simulation"); 
  nh_.param<std::string>("odometry_topic", odom_topic, "/odometry_topic");    
  nh_.param<double>("odometry_frequency", odom_frequency, 100.0);  
  nh_.param<std::string>("pointcloud_topic", pc_topic, "/pointcloud_topic");    
  nh_.param<double>("pointcloud_frequency", pc_frequency, 15.0);  
  nh_.param<std::string>("cmd_topic", cmd_topic, "/cmd_topic"); 
  nh_.param<double>("maxTrajectoryLength", maxR, 6.0); 
  nh_.param<double>("minTrajectoryLength", minR, 3.0);
  nh_.param<double>("fov_x", xFOV, 60.0);
  nh_.param<double>("fov_y", yFOV, 60.0);
  nh_.param<double>("mass", mass, 1.0);
  nh_.param<double>("max_thrust", maxT, 35.0);
  nh_.param<double>("max_velocity", maxV, 10.0);
  nh_.param<double>("max_angularvelocity", maxAngV, 35.0);
  //nh_.param<double>("average_velocity", avgV, 1.0);
  nh_.param<double>("hovering_height", hHover, 1.0);
  nh_.param<std::vector <double>>("initial_position", iPosition, {0, 0, 0.08});
  nh_.param<std::vector <double>>("final_goal", final_goal, {0, 0, 2});   
  //additional parameters for controller
  nh_.param<double>("tau_rp", tau_rp, 1.0);
  nh_.param<double>("tau_yaw", tau_yaw, 1.0);
  nh_.param<std::vector <double>>("kx", kx, {5, 5, 5});
  nh_.param<std::vector <double>>("kv", kv, {4, 4, 4});
  nh_.param<std::vector <double>>("kR", kR, {5, 5, 5});
  nh_.param<std::vector <double>>("kOmega", kOmega, {4, 4, 4});
  // paramters for pointcloud processing
  nh_.param<std::vector <double>>("cogTbase", cgTb, {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
  nh_.param<std::vector <double>>("baseTcoc", bToc, {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});

  // Eigen matrix is filled column wise 
  cogTbase << cgTb[0], cgTb[1], cgTb[2], cgTb[3], 
              cgTb[4], cgTb[5], cgTb[6], cgTb[7], 
              cgTb[8], cgTb[9], cgTb[10], cgTb[11], 
              cgTb[12], cgTb[13], cgTb[14], cgTb[15];

  baseTcoc << bToc[0], bToc[1], bToc[2], bToc[3], 
              bToc[4], bToc[5], bToc[6], bToc[7], 
              bToc[8], bToc[9], bToc[10], bToc[11], 
              bToc[12], bToc[13], bToc[14], bToc[15];

  cogTcoc = cogTbase * baseTcoc;

  delt = 1.0/odom_frequency;
  controlHorizonTime = 1.0/pc_frequency;
  waypointList = {{iPosition[0], iPosition[1], hHover}, {final_goal[0], final_goal[1], final_goal[2]}, {final_goal[0], final_goal[1], 0.1}};

  //waypointList = {{iPosition[0], iPosition[1], hHover}, {iPosition[0], iPosition[1], 0.1}};

}

void planner::initializeSubscribers()
{
  ROS_INFO("Initializing Planner Subscribers");
  odom_sub = nh_.subscribe(odom_topic.c_str(), 1, &planner::odometry_callback, this);  
  pc_sub = nh_.subscribe(pc_topic.c_str(), 1, &planner::pointcloud_callback, this);  
}

void planner::initializePublishers()
{
  ROS_INFO("Initializing Planner Publishers");
  bool latch;
  if (mode ==  "simulation")
  {
    cmd_pub = nh_.advertise<mav_msgs::Actuators>(cmd_topic.c_str(), 1, latch=true);
  }
  else if (mode == "hardware")
  {
    cmd_pub = nh_.advertise<mavros_msgs::AttitudeTarget>(cmd_topic.c_str(), 1, latch=true);
  }
  pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1, latch=true);
  ensemble_pub = nh_.advertise<sensor_msgs::PointCloud2>("/ensemble", 1, latch=true);
  best_traj_pub = nh_.advertise<sensor_msgs::PointCloud2>("/best_trajectory", 1, latch=true);
  selected_traj_pub = nh_.advertise<sensor_msgs::PointCloud2>("/selected_trajectory", 1, latch=true);
}


void planner::checkConvergance(VectorXd& goalState, VectorXd& currentState, bool& waypointReached, double& convergance)
{
  convergance = sqrt(pow(goalState(0)-currentState(0), 2) + pow(goalState(4)-currentState(4), 2) + pow(goalState(8)-currentState(8), 2));

  if (convergance <=0.1 && wp_counter < waypointList.size()-1)
  {
    wp_counter += 1;
    initialState << waypointList[wp_counter-1][0], 0.0, 0.0, 0.0, 
                    waypointList[wp_counter-1][1], 0.0, 0.0, 0.0, 
                    waypointList[wp_counter-1][2], 0.0, 0.0, 0.0;
    goalState << waypointList[wp_counter][0], 0.0, 0.0, 0.0, waypointList[wp_counter][1], 0.0, 0.0, 0.0, waypointList[wp_counter][2], 0.0, 0.0, 0.0;
    waypointReached  = true;
    replanning_starttme = ros::Time::now().toSec();
  }
  else if (convergance > 0.1 || wp_counter >= waypointList.size()-1)
  {
    waypointReached = false;
  }
}

void planner::finalTrajectory(VectorXd& px, VectorXd& py, VectorXd& pz, Vector4d& yaw_coeff, double& T0, double& T)
{
  //outdata.open("generated_trajectory.txt", std::ios::out|std::ios::binary);  

  int n = int(T/delt) + 1;
  VectorXd timePoints(n);
  timePoints[0] = T0;
  for (int i = 0; i < (n - 1); ++i)
  {
    timePoints[i] =  delt * i;
  }
  timePoints[n-1] = T;
  //outdata<< "points:"<<n<<endl;
  double start_time = ros::Time::now().toSec();
  //double start_time = current_time;
  //outdata<< "trajectory publishing starts:"<<ros::Time::now().toSec()<<endl;
  for (int i=0; i<timePoints.size(); i++)
  {
    double t = timePoints(i);
    double _time = start_time + t;
    double x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, phi, phi_r;
    pcl::PointXYZ pt;
    x = px(0)*t*t*t*t*t*t*t/5040 - px(1)*t*t*t*t*t*t/720 + px(2)*t*t*t*t*t/120 - px(3)*t*t*t*t/24 + px(4)*t*t*t/6 + px(5)* t*t/2 + px(6)*t + px(7);
    y = py(0)*t*t*t*t*t*t*t/5040 - py(1)*t*t*t*t*t*t/720 + py(2)*t*t*t*t*t/120 - py(3)*t*t*t*t/24 + py(4)*t*t*t/6 + py(5)* t*t/2 + py(6)*t + py(7);
    z = pz(0)*t*t*t*t*t*t*t/5040 - pz(1)*t*t*t*t*t*t/720 + pz(2)*t*t*t*t*t/120 - pz(3)*t*t*t*t/24 + pz(4)*t*t*t/6 + pz(5)* t*t/2 + pz(6)*t + pz(7);

    vx = px(0)*t*t*t*t*t*t/720 - px(1)*t*t*t*t*t/120 + px(2)*t*t*t*t/24 - px(3)*t*t*t/6 + px(4)*t*t/2 + px(5)* t + px(6);
    vy = py(0)*t*t*t*t*t*t/720 - py(1)*t*t*t*t*t/120 + py(2)*t*t*t*t/24 - py(3)*t*t*t/6 + py(4)*t*t/2 + py(5)* t + py(6);
    vz = pz(0)*t*t*t*t*t*t/720 - pz(1)*t*t*t*t*t/120 + pz(2)*t*t*t*t/24 - pz(3)*t*t*t/6 + pz(4)*t*t/2 + pz(5)* t + pz(6);

    ax = px(0)*t*t*t*t*t/120 - px(1)*t*t*t*t/24 + px(2)*t*t*t/6 - px(3)*t*t/2 + px(4)*t + px(5);
    ay = py(0)*t*t*t*t*t/120 - py(1)*t*t*t*t/24 + py(2)*t*t*t/6 - py(3)*t*t/2 + py(4)*t + py(5);
    az = pz(0)*t*t*t*t*t/120 - pz(1)*t*t*t*t/24 + pz(2)*t*t*t/6 - pz(3)*t*t/2 + pz(4)*t + pz(5);

    phi = yaw_coeff(0) + yaw_coeff(1)*t + yaw_coeff(2) *t*t + yaw_coeff(3) *t*t*t;
    phi_r = yaw_coeff(1) + 2 *yaw_coeff(2) *t + 3 *yaw_coeff(3) *t*t;

    outdata1 << _time <<","<<x<<","<<y<<","<<z<<","<<vx<<","<<vy<<","<<vz<<","<<ax<<","<<ay<<","<<az<<","<<phi<<","<<phi_r<<endl;

    pt.x = x;
    pt.y = y;
    pt.z = z;
    traj_cloud.points.push_back(pt);
    
    traj_global_time.push_back(_time);
    xdes.push_back(x); ydes.push_back(y); zdes.push_back(z);
    vxdes.push_back(vx); vydes.push_back(vy); vzdes.push_back(vz);
    axdes.push_back(ax); aydes.push_back(ay); azdes.push_back(az);
    yawdes.push_back(phi); yawratedes.push_back(phi_r);
    if (traj_global_time.size() >= 2000)
    {
      traj_global_time.erase(traj_global_time.begin());
      xdes.erase(xdes.begin()); ydes.erase(ydes.begin()); zdes.erase(zdes.begin());
      vxdes.erase(vxdes.begin()); vydes.erase(vydes.begin()); vzdes.erase(vzdes.begin());
      axdes.erase(axdes.begin()); aydes.erase(aydes.begin()); azdes.erase(azdes.begin());
      yawdes.erase(yawdes.begin()); yawratedes.erase(yawratedes.begin());
    }
    
  }
  //outdata<< "trajectory publishing ends"<<","<<ros::Time::now().toSec()<<endl;
  trajectoryAvailable = true;
  sensor_msgs::PointCloud2 pc_traj;
  pcl::toROSMsg(traj_cloud, pc_traj);
  pc_traj.header.frame_id = "world";
  pc_traj.header.stamp = ros::Time::now();
  best_traj_pub.publish(pc_traj);
}

void planner::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_time = msg->header.stamp.toSec();
  Eigen::Quaterniond q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3d R = q.toRotationMatrix();
  dronePose = Eigen::Matrix4d::Identity();
  dronePose.block(0,0,3,3) = R;
  dronePose(0,3) = msg->pose.pose.position.x;
  dronePose(1,3) = msg->pose.pose.position.y;
  dronePose(2,3) = msg->pose.pose.position.z;

  // these two lines are needed only for rotorS, because the linear velocity is expressed in body frame
  Eigen::Vector3d Vc = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  Vc = R*Vc;
  // so these lines convert that into inertial frame, not needed eventually for hardware tests.

  if (odom_counter == 0)
  {
    // 0.1 is added to z of initialState because the odometry--due to the noises-- gives negative z velocities
    // and ex and ev in the controller are so small that quad goes slightly downwards and since its already 
    // on the ground, collision is detected and current odometry has nan velocities
    initialState << msg->pose.pose.position.x, Vc(0), 0.0, 0.0, 
                    msg->pose.pose.position.y, Vc(1), 0.0, 0.0, 
                    msg->pose.pose.position.z+0.1, Vc(2), 0.0, 0.0;
    goalState << waypointList[wp_counter][0], 0.0, 0.0, 0.0, waypointList[wp_counter][1], 0.0, 0.0, 0.0, waypointList[wp_counter][2], 0.0, 0.0, 0.0;
  }

  currentState << msg->pose.pose.position.x, Vc(0), 0.0, 0.0, 
                  msg->pose.pose.position.y, Vc(1), 0.0, 0.0, 
                  msg->pose.pose.position.z, Vc(2), 0.0, 0.0;
  

  currentYaw = tf2::getYaw(msg->pose.pose.orientation);
  currentYawrate = 0.0;

  if (trajectoryAvailable)
  {
    
    //auto it = std::lower_bound(traj_global_time.begin(), traj_global_time.end(), current_time);
    //int j = std::distance(traj_global_time.begin(), it);
    //if (j == traj_global_time.size()){j = j-1;}

    int j;
    for (int i = traj_global_time.size()-1; i >= 0; --i)
    {
      if (traj_global_time[i] < current_time)
      {
        j = i;
        break;
      }
      else
      {
        j = traj_global_time.size()-1;
      }
      
    }


    // last two on desired state are yaw and yaw_rate
    desiredState<< xdes[j], vxdes[j], axdes[j], 0.0, ydes[j], vydes[j], aydes[j], 0.0, zdes[j], vzdes[j], azdes[j], 0.0, yawdes[j], yawratedes[j];
    
    outdata<<traj_global_time[j]<<","<<desiredState(0)<<","<<currentState(0)<<","<<desiredState(4)<<","<<currentState(4)<<","<<desiredState(8)
            <<","<<currentState(8)<<","<<desiredState(1)<<","<<currentState(1)<<","<<desiredState(5)<<","<<currentState(5)<<","
            <<desiredState(9)<<","<<currentState(9)<<","<<desiredState(2)<<","<<currentState(2)<<","<<desiredState(6)<<","<<currentState(6)<<","
            <<desiredState(10)<<","<<currentState(10)<<","<<desiredState(12)<<","<<currentYaw<<","<<desiredState(13)<<","<<currentYawrate<<endl;

    if (mode == "simulation")
    {
      controller_simulation control;
      Eigen::VectorXd w(6);
      control.calculateCommands(msg, desiredState, kx, kx, kR, kOmega, mass, w);

      mav_msgs::Actuators cmd_msg;
      cmd_msg.header.stamp = msg->header.stamp;
      for (int i; i< w.size(); i++)
      {
        cmd_msg.angular_velocities.push_back(w(i)); 
      }
      cmd_pub.publish(cmd_msg);
    }
    else if (mode == "hardware")
    {
      controller_px4 control;
      Eigen::VectorXd w(4);
      control.calculateCommands(msg, desiredState, kx, kx, tau_rp, tau_yaw, maxT, mass, w);
      mavros_msgs::AttitudeTarget cmd_msg;
      cmd_msg.header.stamp = msg->header.stamp;
      cmd_msg.type_mask = 128;
      cmd_msg.body_rate.x = w(0);
      cmd_msg.body_rate.y = w(0);
      cmd_msg.body_rate.z = w(0);
      cmd_msg.thrust = w(0);
    }
    

  }
  odom_counter += 1;
}

void planner::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  //cout << "wp_counter: "<< wp_counter<<",republishTrajectory:"<<republishTrajectory<<endl;
  clock_t start, end;
  start = clock();   
  double convergance, time_taken;
  double T0 = 0.0;

  //cout << "size of the cloud is:"<< cloud_in->height * cloud_in->width<<endl;

  //end = clock(); 
  //time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
  //cout << "Time taken 0: " << fixed << time_taken << setprecision(7) << "sec" << endl; 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ensemble_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  vector<vector<double>> points;
  vector<vector<double>> availablepoints;
  
  bool badTrajectory;
  ensembleGeneration ensemble;
  OptimalControlTrajectory trajectory;
  ensemble.processPointcloud(cloud_in, xFOV, yFOV, maxR, cogTcoc, dronePose, cloud_out);




  if (!RHPmode)
  {
    checkConvergance(goalState, currentState, waypointReached, convergance);
    if (abs(goalState(0) - initialState(0))<0.01 && abs(goalState(4) - initialState(4))<0.01)
    {desiredYaw = 0;}
    else {desiredYaw = atan2(goalState(4) - initialState(4), goalState(0) - initialState(0));}
    Eigen::Vector2d yaw_ic, yaw_fc;
    Eigen::Vector4d yaw_coeff;
    yaw_ic << currentYaw, currentYawrate;
    yaw_fc << desiredYaw, 0.0;
    //cout <<"waypointReached:" <<waypointReached<<"pc_counter"<<pc_counter<<endl;
    if (waypointReached || pc_counter == 0 ) // pc_counter condition is added because I am not getting a good logic at very first point
    {
      if (wp_counter == 0 || wp_counter == waypointList.size()-1)
      {
        double hoverMaxV = 1.0;
        trajectory.generateTrajectory(initialState, goalState, hoverMaxV, maxT, maxAngV, mass, px, py, pz, T, badTrajectory);
        trajectory.yawCoeff(yaw_ic, yaw_fc, T, yaw_coeff);
        finalTrajectory(px, py, pz, yaw_coeff, T0, T);
        //ros::Duration(T+1.0).sleep();
      }
      else
      {
        trajectory.generateTrajectory(initialState, goalState, maxV, maxT, maxAngV, mass, px, py, pz, T, badTrajectory);
        trajectory.yawCoeff(yaw_ic, yaw_fc, T, yaw_coeff);
        finalTrajectory(px, py, pz, yaw_coeff, T0, T);
        //ros::Duration(T+1.0).sleep();
      }
      trajectoryAvailable = true;
      end = clock(); 
      time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
      cout << "Time taken in trajectory generation: " << fixed << time_taken << setprecision(7) << "sec" << endl; 
      //ros::Duration(T+1.0).sleep();      
    }
  }
  else
  {
    checkConvergance(goalState, currentState, waypointReached, convergance);

    if (waypointReached || pc_counter == 0 || replan) 
    {
      //cout << "waypoint reached:"<<waypointReached<<endl;
      if (wp_counter == 0 || wp_counter == waypointList.size()-1)
      {
        //cout <<"wp_counter:"<<wp_counter<<", waypointList.size():"<<waypointList.size()<<endl;
        if (wp_counter == waypointList.size()-1 && waypointList.size() != 2)
        {
          initialState << local_goal[0], 0.0, 0.0, 0.0, local_goal[1], 0.0, 0.0, 0.0, local_goal[2], 0.0, 0.0, 0.0;
          goalState << local_goal[0], 0.0, 0.0, 0.0, local_goal[1], 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0;           
        }
        else if(wp_counter == waypointList.size()-1 && waypointList.size() == 2)
        {
          initialState = goalState;
          goalState << goalState(0), 0.0, 0.0, 0.0, goalState(1), 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0;
        }
        //outdata<<"planning started:"<<ros::Time::now().toSec();
        if (abs(goalState(0) - initialState(0))<0.01 && abs(goalState(4) - initialState(4))<0.01)
        {desiredYaw = 0;}
        else 
        {
          desiredYaw = atan2(goalState(4) - initialState(4), goalState(0) - initialState(0));
          //desiredYaw = currentYaw;
        }
        Eigen::Vector2d yaw_ic, yaw_fc;
        Eigen::Vector4d yaw_coeff;
        yaw_ic << currentYaw, currentYawrate;
        yaw_fc << desiredYaw, 0.0;
        double hoverMaxV = 1.0;
        trajectory.generateTrajectory(initialState, goalState, hoverMaxV, maxT, maxAngV, mass, px, py, pz, T, badTrajectory);
        trajectory.yawCoeff(yaw_ic, yaw_fc, T, yaw_coeff);
        finalTrajectory(px, py, pz, yaw_coeff, T0, T);
        //trajectoryAvailable = true;
        //ros::Duration(T+1.0).sleep();
      }
      else
      {        
        //outdata<<"RHP started"<<ros::Time::now().toSec()<<endl;
        /*double kvel = 0.3;
        double factor = erf(kvel * (current_time-replanning_starttme));// in case factor is zero or negative
        double maxVel = max(1.0, factor*maxV) ; 
        ensemble.generateEnsemble(dronePose, xFOV, yFOV, maxR, minR, maxVel, points);
        ensemble.checkcollosionGetcost(cloud_out, initialState, goalState, maxT, maxAngV, mass, points, availablepoints, ensemble_cloud, outdata2);
        cout <<"total available points:"<<availablepoints.size()<<endl;

        if (availablepoints.size() != 0)
        {
          ensemble.generateLocalgoal(availablepoints, local_goal);          
          sensor_msgs::PointCloud2 pc_ensemble;
          pcl::toROSMsg(*ensemble_cloud, pc_ensemble);
          pc_ensemble.header.frame_id = "world";
          pc_ensemble.header.stamp = ros::Time::now();
          ensemble_pub.publish(pc_ensemble);
          localGoal << local_goal[0], 0.0, 0.0, 0.0, local_goal[1], 0.0, 0.0, 0.0, local_goal[2], 0.0, 0.0, 0.0;
        }
        else
        {
          cout <<"no points available in trajectory ensemble"<<endl;
          local_goal[0] = localGoal(0); local_goal[1] = localGoal(1); local_goal[2] = localGoal(2); local_goal[3] = maxVel;
        }*/
        
        convergance = sqrt(pow(goalState(0)-currentState(0), 2) + pow(goalState(4)-currentState(4), 2) + pow(goalState(8)-currentState(8), 2));
        //cout<<"convergance:"<< convergance<<endl;
        if (convergance > maxR)
        {
          Eigen::Vector2d yaw_ic, yaw_fc;
          double kvel = 0.3;
          double factor = erf(kvel * (current_time-replanning_starttme));// in case factor is zero or negative
          double maxVel = max(1.0, factor*maxV) ; 
          ensemble.generateEnsemble(dronePose, xFOV, yFOV, maxR, minR, maxVel, points);
          ensemble.checkcollosionGetcost(cloud_out, initialState, goalState, maxT, maxAngV, mass, points, availablepoints, ensemble_cloud, outdata2);
          cout <<"total available points:"<<availablepoints.size()<<endl;
          

          if (availablepoints.size() != 0) // if there is a path
          {
            ensemble.generateLocalgoal(availablepoints, local_goal);          
            sensor_msgs::PointCloud2 pc_ensemble;
            pcl::toROSMsg(*ensemble_cloud, pc_ensemble);
            pc_ensemble.header.frame_id = "world";
            pc_ensemble.header.stamp = ros::Time::now();
            ensemble_pub.publish(pc_ensemble);
            localGoal << local_goal[0], 0.0, 0.0, 0.0, local_goal[1], 0.0, 0.0, 0.0, local_goal[2], 0.0, 0.0, 0.0;

            if (abs(localGoal(0) - initialState(0))<0.01 && abs(localGoal(4) - initialState(4))<0.01)
            {desiredYaw = 0;}
            else {desiredYaw = atan2(localGoal(4) - initialState(4), localGoal(0) - initialState(0));}


            if (rhp_counter == 0){yaw_ic << currentYaw, currentYawrate;}
            else{yaw_ic << controlHorizonYaw, controlHorizonYawrate;}
            
            yaw_fc << desiredYaw, 0.0;

            trajectory.generateTrajectory(initialState, localGoal, local_goal[3], maxT, maxAngV, mass, px, py, pz, T, badTrajectory);
          
            trajectory.yawCoeff(yaw_ic, yaw_fc, T, yaw_coeff);

            controlHorizonYaw = yaw_coeff(0) + yaw_coeff(1)*controlHorizonTime + yaw_coeff(2)*pow(controlHorizonTime,2) + 
                                yaw_coeff(3)*pow(controlHorizonTime,3);
            controlHorizonYawrate = yaw_coeff(1) + 2 * yaw_coeff(2) * controlHorizonTime + 3 * yaw_coeff(3) * pow(controlHorizonTime,2);

            trajectory.getControlHorizonPoint(px, py, pz, controlHorizonTime, initialState);
            finalTrajectory(px, py, pz, yaw_coeff, T0, controlHorizonTime);
            end = clock(); 
            time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
            //cout << "Time taken 6: " << fixed << time_taken << setprecision(7) << "sec" << endl; 

            replan = true;
            outdata2 <<  cloud_in->height * cloud_in->width <<","<<availablepoints.size()<<","<<fixed << time_taken << setprecision(7)<<endl;
          }
          else // if there is no path
          {
            cout <<"No points are available in the field of view, stopping the quad."<<endl;
            local_goal[0] = localGoal(0); local_goal[1] = localGoal(1); local_goal[2] = localGoal(2); local_goal[3] = maxVel;
            finalTrajectory(px, py, pz, yaw_coeff, controlHorizonTime, T);
            replan = false;
          }
          //outdata<<"RHP finished"<<ros::Time::now().toSec()<<endl;
        }
        else
        {
          goalState << local_goal[0], 0.0, 0.0, 0.0, local_goal[1], 0.0, 0.0, 0.0, local_goal[2], 0.0, 0.0, 0.0;
          if (abs(goalState(0) - initialState(0))<0.01 && abs(goalState(4) - initialState(4))<0.01)
          {desiredYaw = 0;}
          else {desiredYaw = atan2(goalState(4) - initialState(4), goalState(0) - initialState(0));}
          Eigen::Vector2d yaw_ic, yaw_fc;
          Eigen::Vector4d yaw_coeff;
          yaw_ic << controlHorizonYaw, controlHorizonYawrate;
          yaw_fc << desiredYaw, 0.0;

          double v = local_goal[3]/2;
          trajectory.generateTrajectory(initialState, goalState, v, maxT, maxAngV, mass, px, py, pz, T, badTrajectory);
          trajectory.yawCoeff(yaw_ic, yaw_fc, T, yaw_coeff);
          //trajectory.getControlHorizonPoint(px, py, pz, controlHorizonTime, initialState);
          finalTrajectory(px, py, pz, yaw_coeff, T0, T);
          replan = false;
          //ros::Duration(T+1.0).sleep();
        }       
        rhp_counter += 1; 
        int n = int(T/delt) + 1;
        VectorXd timePoints(n);
        timePoints[0] = 0.0;
        for (int i = 0; i < (n - 1); ++i)
        {
          timePoints[i] =  delt * i;
        }
        timePoints[n-1] = T;
        for (int i=0; i<timePoints.size(); i++)
        {
          double t = timePoints(i);
          double x, y, z;
          pcl::PointXYZ _pt;
          x = px(0)*t*t*t*t*t*t*t/5040 - px(1)*t*t*t*t*t*t/720 + px(2)*t*t*t*t*t/120 - px(3)*t*t*t*t/24 + px(4)*t*t*t/6 + px(5)* t*t/2 + px(6)*t + px(7);
          y = py(0)*t*t*t*t*t*t*t/5040 - py(1)*t*t*t*t*t*t/720 + py(2)*t*t*t*t*t/120 - py(3)*t*t*t*t/24 + py(4)*t*t*t/6 + py(5)* t*t/2 + py(6)*t + py(7);
          z = pz(0)*t*t*t*t*t*t*t/5040 - pz(1)*t*t*t*t*t*t/720 + pz(2)*t*t*t*t*t/120 - pz(3)*t*t*t*t/24 + pz(4)*t*t*t/6 + pz(5)* t*t/2 + pz(6)*t + pz(7);      
          _pt.x = x;
          _pt.y = y;
          _pt.z = z;
          selected_traj_cloud.points.push_back(_pt);      
          
        }
      }

      sensor_msgs::PointCloud2 final_traj;
      pcl::toROSMsg(selected_traj_cloud, final_traj);
      final_traj.header.frame_id = "world";
      final_traj.header.stamp = ros::Time::now();
      selected_traj_pub.publish(final_traj);

      trajectoryAvailable = true;
      end = clock(); 
      time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
      cout << "Time taken in trajectory generation: " << fixed << time_taken << setprecision(7) << "sec" << endl;     

      
    }

  }
  

  
  //end = clock(); 
  //double time_taken2 = double(end - start) / double(CLOCKS_PER_SEC); 
  //cout << "Time taken in trajectory generation: " << fixed << time_taken2 << setprecision(7) << "sec" << endl; 
  pc_counter += 1;  


}

    

