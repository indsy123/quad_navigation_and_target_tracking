#include "ensembleGeneration.h"

using namespace std;
using namespace Eigen;

void ensembleGeneration::processPointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in, double& xFOV, double& yFOV, double& maxR, 
                                          Matrix4d& cogTcoc, Matrix4d& dronePose, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*cloud_in, *pcl_cloud);
  //cout <<"N_inp_cloud_pcl"<<pcl_cloud->points.size()<<endl;
  Eigen::Matrix4d transform = dronePose * cogTcoc;
  pcl::transformPointCloud (*pcl_cloud, *cloud_out, transform);
  //pcl::transformPointCloud (*pcl_cloud, *transformed_cloud, cogTcoc);
  //cout <<"N_transformed_cloud"<<cloud_out.points.size()<<endl;

  /*float xfov, yfov, max_r;
  xfov = float(xFOV);
  yfov = float(yFOV);
  max_r = float(maxR);

  pcl::FrustumCulling<pcl::PointXYZ> fc;
  fc.setInputCloud (transformed_cloud);
  fc.setVerticalFOV (yfov);
  fc.setHorizontalFOV (xfov);
  //fc.setNearPlaneDistance (0.0);
  //fc.setFarPlaneDistance (max_r);

  // .. read or input the camera pose from a registration algorithm.
  //fc.setCameraPose(dronePose);
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr truncated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fc.filter (cloud_out);
  cout <<"N_cloud_out"<<cloud_out.points.size()<<endl;*/

}

void ensembleGeneration::generateEnsemble(Matrix4d& dronePose, double& xFOV, double& yFOV, double& maxR, double& minR, double& maxVelocity,
                                      vector<vector<double>>& points)
{
  // hardcoded paramters for ensamble generation, can be optimized as per user's need
  // since pointcloud is in global frame xFOV is not in y-direction and yFOV is in z-direction
  int nPlanes = 3; 
  double d = (maxR-minR)/(nPlanes-1);
  double spacing = 0.25;
  //int delTheta = 10;
  int xHalfangle = int(0.5*xFOV);
  int yHalfangle = int(0.5*yFOV);
  // hardcoded paramters for ensamble generation, can be optimized as per user's need
  
  double R, trajVel, arcx, arcy, thetax, thetay;
  int Nx, Ny;
  Eigen::Vector4d point, _point;
  double _PI = PI/180.0;
  int index;
  for (int i = 0; i < nPlanes; ++i)
  {
    if (i % 2 == 0){index = 0;}
    else{index = 1;} 
    R = minR + d*i;
    arcx = R * xFOV*_PI;
    arcy = R * yFOV*_PI;
    Nx = int(arcx/spacing)+1;
    Ny = int(arcy/spacing)+1;
    trajVel = maxVelocity * (R/maxR);
    for (int j = 0; j <= Nx; ++j)
    {
      thetax = -xHalfangle + (xFOV/Nx)*j + 5 * index;
      for (int k = 0; k <= Ny; ++k)
      {        
        vector<double> p(4,0);
        thetay = -yHalfangle + (yFOV/Ny)*k + 5 * index;
        _point << R*cos(thetax*_PI)*cos(thetay*_PI), R*sin(thetax*_PI)*cos(thetay*_PI), R*sin(thetay*_PI), 1.0; 
        point = dronePose * _point;
        p = {point(0), point(1), point(2), trajVel};
	//points.push_back(p); 
        if (p[2] > 0.25) 
        {points.push_back(p);}               
      }
    } 
  }
  //cout << "points in the ensemble:"<<points.size()<<endl;

}


void ensembleGeneration::checkcollosionGetcost(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, VectorXd& initialState, VectorXd& goalState, double& maxThrust, double& maxAngularVelocity, double& mass, vector< vector<double>>& points,                         vector< vector<double>>& availablepoints, pcl::PointCloud<pcl::PointXYZ>::Ptr& ensamble_cloud, std::ofstream& outdata2)
{
  //double start, end;
  //start = clock();

  bool EmptyPointCloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  if (cloud->points.size() != 0)
  {
    EmptyPointCloud = false;
    kdtree.setInputCloud (cloud);    
  }
  else
  {
    EmptyPointCloud = true;
  }


  if (!EmptyPointCloud) // when the obstacles are there in the field of view
  {
    double T, t, r, time_taken; 
    OptimalControlTrajectory trajectory;
    double _a, _b, _c, _d, _e, _f;
    _a = 1.0/5040; _b = 1.0/720; _c = 1.0/120; _d = 1.0/24; _e = 1.0/6; _f = 0.5;
    double robot_radius = 0.3;
    double SR = 0.6;
    double factor = (1 + SR*SR*SR*SR)/(SR*SR*SR*SR);

    //cout << "goal state:"<<goalState(0)<<","<<goalState(4)<<","<<goalState(8)<<endl;
    for (int i = points.size(); i--;)
    {
      VectorXd trajectoryEndpoint(12);
      trajectoryEndpoint << points[i][0], 0.0, 0.0, 0.0, points[i][1], 0.0, 0.0, 0.0, points[i][2], 0.0, 0.0, 0.0;


      VectorXd px(8), py(8), pz(8);
      bool badTrajectory;
      trajectory.generateTrajectory(initialState, trajectoryEndpoint, points[i][3], maxThrust, maxAngularVelocity, mass, 
                                    px, py, pz, T, badTrajectory); 

      //end = clock(); 
      //time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
      //cout << "TT1: " << fixed << time_taken << setprecision(7) << "sec" << endl;

      double trajLength = sqrt((initialState(0)-points[i][0])*(initialState(0)-points[i][0]) +
                               (initialState(4)-points[i][1])*(initialState(4)-points[i][1]) +
                               (initialState(8)-points[i][2])*(initialState(8)-points[i][2]));
      
      int interval = int(0.5*trajLength/0.2) + 5; // 5 is added to increase the number of points

      std::vector<float> distance_with_obstacles(interval);

      for (int p = 0; p < distance_with_obstacles.size(); p++)
      {
        distance_with_obstacles[p] = 0.0;
      }
      
      bool intersection_detected = false;
      for (int k = interval; k--;)
      {
        t = T*k/interval;
        //std::vector<float> point(3);
        pcl::PointXYZ pts_;
        pts_.x = px(0)*t*t*t*t*t*t*t*_a - px(1)*t*t*t*t*t*t*_b + px(2)*t*t*t*t*t*_c - px(3)*t*t*t*t*_d + px(4)*t*t*t*_e + px(5)* t*t*_f + px(6)*t + px(7);
        pts_.y = py(0)*t*t*t*t*t*t*t*_a - py(1)*t*t*t*t*t*t*_b + py(2)*t*t*t*t*t*_c - py(3)*t*t*t*t*_d + py(4)*t*t*t*_e + py(5)* t*t*_f + py(6)*t + py(7);
        pts_.z  = pz(0)*t*t*t*t*t*t*t*_a - pz(1)*t*t*t*t*t*t*_b + pz(2)*t*t*t*t*t*_c - pz(3)*t*t*t*t*_d + pz(4)*t*t*t*_e + pz(5)* t*t*_f + pz(6)*t + pz(7);
        
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if ( kdtree.radiusSearch (pts_, robot_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          //cout <<"intersection detected"<<endl;
          intersection_detected = true;
          break;
        }  
        int N = 1;
        std::vector<int> pointIdxNKNSearch(N);
        std::vector<float> pointNKNSquaredDistance(N);
        
        if ( kdtree.nearestKSearch (pts_, N, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        { 
          float dd = sqrt(pointNKNSquaredDistance[0]); // distance of this point to nearest obstacle
          distance_with_obstacles[k] = dd;
          //float normalized_dd;
          //if (dd - robot_radius < SR)
          //{
          //  normalized_dd = factor * ((dd-robot_radius)*(dd-robot_radius) - SR*SR)*((dd-robot_radius)*(dd-robot_radius) - SR*SR)/
          //                  (1 + ((dd-robot_radius)*(dd-robot_radius) - SR*SR)*((dd-robot_radius)*(dd-robot_radius) - SR*SR));
          //}
          //else{normalized_dd = 0.0;}     
          //normalized_dd = 1.0/dd;
          //normalized_dd = exp(-dd);
          //distance_with_obstacles.push_back(normalized_dd);//sqrt(pointNKNSquaredDistance[0]); 
        }         
      }

      //end = clock(); 
      //time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
      //cout << "TT2: " << fixed << time_taken << setprecision(7) << "sec" << endl;
      //vector<double>a(6);
      //if (intersection_detected == false && !badTrajectory)
      if (intersection_detected == false && !badTrajectory)
      { 
               
        // vector a contains x, y coordinate of the point, distance2goal cost and obstacle cost 
        vector<double>a(6);
        double mindist2obs = *min_element(distance_with_obstacles.begin(), distance_with_obstacles.end());
        double scaled_mindist;
        if (mindist2obs - robot_radius < SR)
        {
          scaled_mindist = factor * ((mindist2obs-robot_radius)*(mindist2obs-robot_radius) - SR*SR)*((mindist2obs-robot_radius)*(mindist2obs-robot_radius) - SR*SR)/
                          (1 + ((mindist2obs-robot_radius)*(mindist2obs-robot_radius) - SR*SR)*((mindist2obs-robot_radius)*(mindist2obs-robot_radius) - SR*SR));
        }
        else{scaled_mindist = 0.0;}

        //cout <<"scaled_mindist"<<scaled_mindist<<endl;



        a = {points[i][0], points[i][1], points[i][2], points[i][3], 
            (goalState(0)-points[i][0])*(goalState(0)-points[i][0]) + 
            (goalState(4)-points[i][1])*(goalState(4)-points[i][1]) +
            (goalState(8)-points[i][2])*(goalState(8)-points[i][2]),
            scaled_mindist}; 
            //std::accumulate(distance_with_obstacles.begin(), distance_with_obstacles.end(), 0.0)};
        //cout << "obstacle_cost:"<< a[5]<<endl;
        //a[5] = *min_element(distance_with_obstacles.begin(), distance_with_obstacles.end()); // min element

        for (int k = interval-1; k--;)
        {
          pcl::PointXYZ _pts;
          double t = T*(k+1)/interval;
          //std::vector<float> point(3);
          _pts.x = px(0)*t*t*t*t*t*t*t*_a - px(1)*t*t*t*t*t*t*_b + px(2)*t*t*t*t*t*_c - px(3)*t*t*t*t*_d + px(4)*t*t*t*_e + px(5)* t*t*_f + px(6)*t + px(7);
          _pts.y = py(0)*t*t*t*t*t*t*t*_a - py(1)*t*t*t*t*t*t*_b + py(2)*t*t*t*t*t*_c - py(3)*t*t*t*t*_d + py(4)*t*t*t*_e + py(5)* t*t*_f + py(6)*t + py(7);
          _pts.z  = pz(0)*t*t*t*t*t*t*t*_a - pz(1)*t*t*t*t*t*t*_b + pz(2)*t*t*t*t*t*_c - pz(3)*t*t*t*t*_d + pz(4)*t*t*t*_e + pz(5)* t*t*_f + pz(6)*t + pz(7);
          ensamble_cloud->points.push_back(_pts);
        }       
        
        availablepoints.push_back(a);
      //end = clock(); 
      //time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
      //cout << "TT3: " << fixed << time_taken << setprecision(7) << "sec" << endl;
      }      
    }
  }
  else // when there is no obstacles in the field of view, obtacle cost of all trajectories are zero 
  {
    for (int i = 0; i < points.size(); i++)
    {
      vector<double>a(6);
      a[0] = points[i][0];
      a[1] = points[i][1];
      a[2] = points[i][2];
      a[3] = points[i][3];
      a[4] = (goalState(0)-points[i][0])*(goalState(0)-points[i][0]) + 
            (goalState(4)-points[i][1])*(goalState(4)-points[i][1]) +
            (goalState(8)-points[i][2])*(goalState(8)-points[i][2]);     
      a[5] = 0.0;  
      availablepoints.push_back(a);      
    }
  }
}

void ensembleGeneration::generateLocalgoal(vector< vector<double>>& availablepoints,  vector<double>& local_goal)
{

  double fd = 1.0;
  double fc = 0.1;

  //cout<< "total available points:"<<availablepoints.size()<<endl;

  vector<double> dist2finalgoal(availablepoints.size());
  vector<double> dist2goal_cost(availablepoints.size());
  vector<double> collision_cost(availablepoints.size());
  vector<double> total_cost(availablepoints.size());

  for (int i = 0; i < availablepoints.size(); i++)
  {
    dist2finalgoal[i] = availablepoints[i][4];
    collision_cost[i] = availablepoints[i][5];
  }

  vector<double> intermediate_point(3);
  int IntermediatepointIndex = std::min_element(dist2finalgoal.begin(),dist2finalgoal.end()) - dist2finalgoal.begin();
  intermediate_point[0] = availablepoints[IntermediatepointIndex][0];
  intermediate_point[1] = availablepoints[IntermediatepointIndex][1];
  intermediate_point[2] = availablepoints[IntermediatepointIndex][2];

  for (int i = 0; i < availablepoints.size(); i++)
  {
    dist2goal_cost[i] = sqrt((intermediate_point[0]-availablepoints[i][0])*(intermediate_point[0]-availablepoints[i][0]) + 
                             (intermediate_point[1]-availablepoints[i][1])*(intermediate_point[1]-availablepoints[i][1]) +
                             (intermediate_point[2]-availablepoints[i][2])*(intermediate_point[2]-availablepoints[i][2]));
  }

  double max_dist2goal_cost = *max_element(dist2goal_cost.begin(), dist2goal_cost.end());
  double max_collision_cost = *max_element(collision_cost.begin(), collision_cost.end());
  double factored_max_dist2goal_cost_inv = fd/max_dist2goal_cost;
  double factored_max_collision_cost_inv = fc/max_collision_cost;

  //cout <<"max_dist2goal_cost:"<<max_dist2goal_cost<<","<<"max_collision_cost:"<<max_collision_cost<<endl;

  for (int i = 0; i < availablepoints.size(); i++)
  {
    if (max_collision_cost != 0.0)
    {
      total_cost[i] = dist2goal_cost[i] * factored_max_dist2goal_cost_inv + collision_cost[i] * factored_max_collision_cost_inv;
    }
    else
    {
       total_cost[i] = dist2goal_cost[i] * factored_max_dist2goal_cost_inv;
    }

  }

  int minElementIndex = std::min_element(total_cost.begin(),total_cost.end()) - total_cost.begin();
  local_goal[0] = availablepoints[minElementIndex][0];
  local_goal[1] = availablepoints[minElementIndex][1];
  local_goal[2] = availablepoints[minElementIndex][2];
  local_goal[3] = availablepoints[minElementIndex][3];
  //cout << "index of min cost" << minElementIndex<<endl;

}


