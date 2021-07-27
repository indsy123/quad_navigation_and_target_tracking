#ifndef ENSEMBLEGENERATION_H
#define ENSEMBLEGENERATION_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include <vector>
#include <numeric>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf2/utils.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <trajectoryGenerationOptimalControl.h>
#include <pcl/filters/frustum_culling.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace Eigen;

class ensembleGeneration
{ 
  public: 
    ensembleGeneration() 
    {
      outdata.open("ensemble.txt", std::ios::out|std::ios::binary);
    };

    void generateEnsemble(Matrix4d& dronePose, double& xFOV, double& yFOV, double& maxR, double& minR, double& maxVelocity,
                          vector<vector<double>>& points);

    void checkcollosionGetcost(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, VectorXd& initialState, VectorXd& goalState,
                              double& maxThrust, double& maxAngularVelocity, double& mass, vector< vector<double>>& points, 
                              vector< vector<double>>& availablepoints, pcl::PointCloud<pcl::PointXYZ>::Ptr& ensamble_cloud, std::ofstream& outdata2);



    void generateLocalgoal(vector< vector<double>>& availablepoints, vector<double>& local_goal);
    void processPointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in, double& xFOV, double& yFOV, double& maxR, 
                            Matrix4d& cogTcoc, Matrix4d& dronePose, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

  private: 
    double PI = 3.1416;

    std::ofstream outdata;



};
#endif //ENSEMBLEGENERATION_H
