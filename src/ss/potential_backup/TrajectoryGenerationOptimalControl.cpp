#include "TrajectoryGenerationOptimalControl.h"

using namespace std;
using namespace Eigen;
//using std::ofstream;


Eigen::VectorXd OptimalControlTrajectory::linspace(double& start, double& end, const int& num)
// not required, delete it eventually
{
  VectorXd linspaced;
  if (num != 0)
  {
    if (num == 1) 
    {
      linspaced(0) = start;
    }
    else
    {
      double delta = (end - start) / (num - 1);
      for (int i = 0; i < (num - 1); ++i)
      {
        linspaced(i) = start + delta * i;
      }
      // ensure that start and end are exactly the same as the input
      linspaced(num-1) = end;
    }
  }
  return linspaced;
}

void OptimalControlTrajectory::yawCoeff(Vector2d& yaw_ic, Vector2d& yaw_fc, double& T, Vector4d& yaw_coeff)
{
  yaw_coeff(0)= yaw_ic[0];
  yaw_coeff(1) = yaw_ic[1];
  yaw_coeff(2) = -0.5 * (-(6.0/(T*T)) * (yaw_fc[0]-yaw_ic[0]) + 2.0/T * (2*yaw_ic[1]+yaw_fc[1]));
  yaw_coeff(3) = 1.0/6 * ((-12.0/(T*T*T)) * (yaw_fc[0]-yaw_ic[0]) + (6.0/(T*T)) * (yaw_ic[1]+yaw_fc[1]));

}

void OptimalControlTrajectory::getOptimalT(VectorXd& start_point, VectorXd& end_point, double& maxVelocity, double& T)
{
  double k, a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3;
  k = 8.8074e-3*pow(maxVelocity, 10.1143);
  //k = 10*pow(maxVelocity, 10.1143);
  //cout << "the value of k is:" <<k <<endl;
  a1 = 840.0 * (start_point(0) - end_point(0)); 
  a2 = 840.0 * (start_point(4) - end_point(4)); 
  a3 = 840.0 * (start_point(8) - end_point(8));
  b1 = 360.0 * start_point(1); 
  b2 = 360.0 * start_point(5); 
  b3 = 360.0 * start_point(9);
  c1 = 60.0 * start_point(2); 
  c2 = 60.0 * start_point(6); 
  c3 = 60.0 * start_point(10);
  d1 = 4.0 * start_point(3); 
  d2 = 4.0 * start_point(7); 
  d3 = 4.0 * start_point(11);

  //NOTE: the order of polynomial coefficients in GSL is reversed than that of np.root in python
  double p[9];
  double z[16];
  p[8] = -2*k;
  p[7] = 0;
  p[6] = d1*d1 + d2*d2 + d3*d3;
  p[5] = 2 * (c1*d1 + c2*d2 + c3*d3);
  p[4] = c1*c1 + c2*c2 + c3*c3 + 2 * (b1*d1 + b2*d2 + b3*d3);
  p[3] = 2 * (a1*d1 + b1*c1 + a2*d2 + b2*c2 + a3*d3 + b3*c3);
  p[2] = b1*b1 + b2*b2 + b3*b3 + 2 * (a1*c1 + a2*c2 + a3*c3);
  p[1] = 2 * (a1*b1 + a2*b2 + a3*b3);
  p[0] = a1*a1 + a2*a2 + a3*a3;

  //cout << "poly coeffs: "<< p[0]<<","<< p[1]<<","<< p[2]<<","<< p[3]<<","<< p[4]<<","<< p[5]<<","<< p[6]<<","<< p[7]<<","<< p[8]<<endl;


  gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc (9);
  gsl_poly_complex_solve (p, 9, w, z);
  gsl_poly_complex_workspace_free (w);
  for (int i=0; i < 8; i++)
  {
    //cout << i << ":"<<z[2*i] <<", "<< z[2*i+1] <<endl;
    if (z[2*i+1] == 0 && z[2*i] > 0)
    {      
      T = z[2*i];
    }
  }
  /*double dist_to_goal, TT;
  dist_to_goal = sqrt((start_point(0) - end_point(0))*(start_point(0) - end_point(0)) + 
                      (start_point(4) - end_point(4))*(start_point(4) - end_point(4)) + 
                      (start_point(8) - end_point(8))*(start_point(8) - end_point(8)));
  TT = 2*dist_to_goal/maxVelocity;

  double kk;
  kk = 0.5*(p[6]*pow(TT, 6) + p[5]*pow(TT, 5) + p[4]*pow(TT, 4) + p[3]*pow(TT, 3) + p[2]*pow(TT, 2) + p[1] *TT +p[0])/pow(TT, 8);  

  outdata2 << maxVelocity<<","<<T<<","<<k<<","<<TT<<","<<kk<<endl;*/

}

void OptimalControlTrajectory::minSnapCoeff(VectorXd& start_point, VectorXd& end_point, double& T, VectorXd& coefficients) 
{
  double x0, v0, a0, j0, xT, vT, aT, jT;
  x0 = start_point(0);
  v0 = start_point(1);
  a0 = start_point(2);
  j0 = start_point(3);
  xT = end_point(0);
  vT = end_point(1);
  aT = end_point(2);
  jT = end_point(3);

  Eigen::Matrix4d A;
  Eigen::Vector4d delta, first_four;

  A(0,0) = -100800;
  A(0,1) = 50400*T;
  A(0,2) = -10080*T*T;
  A(0,3) = 840*T*T*T;
  A(1,0) = -50400*T;
  A(1,1) = 24480*T*T;
  A(1,2) = -4680*T*T*T;
  A(1,3) = 360*T*T*T*T;
  A(2,0) = -10080*T*T;
  A(2,1) = 4680*T*T*T;
  A(2,2) = -840*T*T*T*T;
  A(2,3) = 60*T*T*T*T*T;
  A(3,0) = -840*T*T*T;
  A(3,1) = 360*T*T*T*T;
  A(3,2) = -60*T*T*T*T*T;
  A(3,3) = 4*T*T*T*T*T*T;

  A = A/(T*T*T*T*T*T*T);

  delta << xT - (x0 + v0 * T + 0.5 * a0 * T*T + j0*T*T*T/6),
            vT - (v0 + a0 * T + j0*T*T/2), 
            aT - (a0+j0*T), 
            jT - j0;
  
  first_four = A*delta;
  coefficients <<first_four(0), first_four(1), first_four(2), first_four(3), j0, a0, v0, x0;
}       
        
void OptimalControlTrajectory::generateTrajectory(VectorXd& start_point, VectorXd& end_point, double& maxVelocity,
double& maxThrust, double& maxAngularVelocity, double& mass, VectorXd& px, VectorXd& py, VectorXd& pz, double& T, bool& badTrajectory)
{
  bool regenerate_trajectory = true;
  bool trajectory_is_feasible = true;
  bool limit_exceeded = false;
  badTrajectory = false;


  VectorXd x_ic(4), y_ic(4), z_ic(4), x_fc(4), y_fc(4), z_fc(4);
  x_ic << start_point(0), start_point(1),start_point(2), start_point(3);
  x_fc << end_point(0), end_point(1), end_point(2), end_point(3);
  y_ic << start_point(4), start_point(5), start_point(6),start_point(7);
  y_fc << end_point(4), end_point(5), end_point(6), end_point(7);
  z_ic << start_point(8), start_point(9),start_point(10), start_point(11);
  z_fc << end_point(8), end_point(9), end_point(10), end_point(11);

  double optimalT;
  OptimalControlTrajectory::getOptimalT(start_point, end_point, maxVelocity, optimalT);
  T = optimalT;

  int n = 15; // points on the trajectory used in all the calculations including start and end point,
  VectorXd timePoints(n);

  /*double dist_to_goal;
  dist_to_goal = sqrt((x_fc(0) - x_ic(0))*(x_fc(0) - x_ic(0)) + (y_fc(0) - y_ic(0))*(y_fc(0) - y_ic(0)) + (z_fc(0) - z_ic(0))*(z_fc(0) - z_ic(0)));
  T = 2*dist_to_goal/maxVelocity;
  optimalT = T;*/

  // what if the initial speed is higher than the specified max speed, this should not happen but lets see what happens when we
  double initialSpeed = sqrt(start_point(1)*start_point(1)+start_point(5)*start_point(5)+start_point(9)*start_point(9));
  //maxVelocity = max(maxVelocity, initialSpeed);

  while (regenerate_trajectory)
  {

    double delta = T / (n - 1);

    timePoints[0] = 0.0;
    for (int i = 0; i < (n - 1); ++i)
    {
      timePoints[i] =  delta * i;
    }
    timePoints[n-1] = T;

    OptimalControlTrajectory::minSnapCoeff(x_ic, x_fc, T, px); 
    OptimalControlTrajectory::minSnapCoeff(y_ic, y_fc, T, py); 
    OptimalControlTrajectory::minSnapCoeff(z_ic, z_fc, T, pz);

    VectorXd v(n), thrust(n), angularVelocity(n);
    
    for (int i=0; i<timePoints.size(); i++)
    {
      double vx, vy, vz, ax, ay, az, jx, jy, jz, j;
      double t = timePoints[i];
      vx = px(0)*t*t*t*t*t*t/720 - px(1)*t*t*t*t*t/120 + px(2)*t*t*t*t/24 - px(3)*t*t*t/6 + px(4)*t*t/2 + px(5)* t + px(6);
      vy = py(0)*t*t*t*t*t*t/720 - py(1)*t*t*t*t*t/120 + py(2)*t*t*t*t/24 - py(3)*t*t*t/6 + py(4)*t*t/2 + py(5)* t + py(6);
      vz = pz(0)*t*t*t*t*t*t/720 - pz(1)*t*t*t*t*t/120 + pz(2)*t*t*t*t/24 - pz(3)*t*t*t/6 + pz(4)*t*t/2 + pz(5)* t + pz(6);

      ax = px(0)*t*t*t*t*t/120 - px(1)*t*t*t*t/24 + px(2)*t*t*t/6 - px(3)*t*t/2 + px(4)*t + px(5);
      ay = py(0)*t*t*t*t*t/120 - py(1)*t*t*t*t/24 + py(2)*t*t*t/6 - py(3)*t*t/2 + py(4)*t + py(5);
      az = pz(0)*t*t*t*t*t/120 - pz(1)*t*t*t*t/24 + pz(2)*t*t*t/6 - pz(3)*t*t/2 + pz(4)*t + pz(5);

      jx = px(0)*t*t*t*t/24 - px(1)*t*t*t/6 + px(2)*t*t/2 - px(3)*t + px(4);
      jy = py(0)*t*t*t*t/24 - py(1)*t*t*t/6 + py(2)*t*t/2 - py(3)*t + py(4);
      jz = pz(0)*t*t*t*t/24 - pz(1)*t*t*t/6 + pz(2)*t*t/2 - pz(3)*t + pz(4);

      v(i) = sqrt(vx*vx + vy*vy + vz*vz);
      //a = np.sqrt(ax*ax + ay*ay + az*az);
      j = sqrt(jx*jx + jy*jy + jz*jz);
      thrust(i) = mass * sqrt(ax*ax + ay*ay + (az+9.8)*(az+9.8));
      angularVelocity(i) = j/thrust(i);
    }

    bool condition;
    if (initialSpeed < maxVelocity)
    {
      condition = v.maxCoeff() > maxVelocity || thrust.maxCoeff() > maxThrust ||  angularVelocity.maxCoeff() > maxAngularVelocity;
    }
    else
    {
      condition = thrust.maxCoeff() > maxThrust ||  angularVelocity.maxCoeff() > maxAngularVelocity;
    }
    
    if (condition)
    {
      T = T * 1.1;
      if (T < optimalT *1.4)
      {
        regenerate_trajectory = true;
        trajectory_is_feasible = false;
        limit_exceeded = true;
      }
      else
      {
        //cout<< "operating point is close to inflection point, returning an infeasible trajectory, check boundary conditions."<< endl;
        regenerate_trajectory = false;
        badTrajectory = true;
      }
    }
    else
    {
      //regenerate_trajectory = false;
      if (!limit_exceeded)
      {
        T = T*0.9;
        regenerate_trajectory = true;
      }
      else
      {
        regenerate_trajectory = false;
      }      
    }
  }
}


void OptimalControlTrajectory::getControlHorizonPoint(VectorXd &px,VectorXd &py, VectorXd &pz, double& t, VectorXd& p)
{
  p(0) = px(0)*t*t*t*t*t*t*t/5040 - px(1)*t*t*t*t*t*t/720 + px(2)*t*t*t*t*t/120 - px(3)*t*t*t*t/24 + px(4)*t*t*t/6 + px(5)* t*t/2 + px(6)*t + px(7);
  p(1) = px(0)*t*t*t*t*t*t/720 - px(1)*t*t*t*t*t/120 + px(2)*t*t*t*t/24 - px(3)*t*t*t/6 + px(4)*t*t/2 + px(5)* t + px(6);
  p(2) = px(0)*t*t*t*t*t/120 - px(1)*t*t*t*t/24 + px(2)*t*t*t/6 - px(3)*t*t/2 + px(4)*t + px(5);
  p(3) = px(0)*t*t*t*t/24 - px(1)*t*t*t/6 + px(2)*t*t/2 - px(3)*t + px(4);

  p(4) = py(0)*t*t*t*t*t*t*t/5040 - py(1)*t*t*t*t*t*t/720 + py(2)*t*t*t*t*t/120 - py(3)*t*t*t*t/24 + py(4)*t*t*t/6 + py(5)* t*t/2 + py(6)*t + py(7);
  p(5) = py(0)*t*t*t*t*t*t/720 - py(1)*t*t*t*t*t/120 + py(2)*t*t*t*t/24 - py(3)*t*t*t/6 + py(4)*t*t/2 + py(5)* t + py(6);
  p(6) = py(0)*t*t*t*t*t/120 - py(1)*t*t*t*t/24 + py(2)*t*t*t/6 - py(3)*t*t/2 + py(4)*t + py(5);
  p(7) = py(0)*t*t*t*t/24 - py(1)*t*t*t/6 + py(2)*t*t/2 - py(3)*t + py(4);

  p(8) = pz(0)*t*t*t*t*t*t*t/5040 - pz(1)*t*t*t*t*t*t/720 + pz(2)*t*t*t*t*t/120 - pz(3)*t*t*t*t/24 + pz(4)*t*t*t/6 + pz(5)* t*t/2 + pz(6)*t + pz(7);  
  p(9) = pz(0)*t*t*t*t*t*t/720 - pz(1)*t*t*t*t*t/120 + pz(2)*t*t*t*t/24 - pz(3)*t*t*t/6 + pz(4)*t*t/2 + pz(5)* t + pz(6);  
  p(10) = pz(0)*t*t*t*t*t/120 - pz(1)*t*t*t*t/24 + pz(2)*t*t*t/6 - pz(3)*t*t/2 + pz(4)*t + pz(5);  
  p(11) = pz(0)*t*t*t*t/24 - pz(1)*t*t*t/6 + pz(2)*t*t/2 - pz(3)*t + pz(4);
}





