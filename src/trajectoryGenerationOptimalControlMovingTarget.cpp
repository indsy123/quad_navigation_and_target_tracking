#include "trajectoryGenerationOptimalControl.h"

using namespace std;
using namespace Eigen;
//using std::ofstream;


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
  //k = 1.3095374 * pow(maxVelocity, 8.604645);

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


  double _T = 1/T;

  A << -100800*_T*_T*_T*_T*_T*_T*_T, 50400*_T*_T*_T*_T*_T*_T, -10080*_T*_T*_T*_T*_T, 840*_T*_T*_T*_T,
       -50400*_T*_T*_T*_T*_T*_T, 24480*_T*_T*_T*_T*_T, -4680*_T*_T*_T*_T, 360*_T*_T*_T, 
       -10080*_T*_T*_T*_T*_T, 4680*_T*_T*_T*_T, -840*_T*_T*_T, 60*_T*_T,
       -840*_T*_T*_T*_T, 360*_T*_T*_T, -60*_T*_T, 4*_T;


  delta << xT - (x0 + v0 * T + 0.5 * a0 * T*T + j0*T*T*T/6),
            vT - (v0 + a0 * T + 0.5*j0*T*T), 
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
  double dist_to_goal;

  dist_to_goal = sqrt((x_fc(0) - x_ic(0))*(x_fc(0) - x_ic(0)) + (y_fc(0) - y_ic(0))*(y_fc(0) - y_ic(0)) + (z_fc(0) - z_ic(0))*(z_fc(0) - z_ic(0)));
  T = 2*dist_to_goal/maxVelocity;
  optimalT = T;  


  /*if (maxVelocity >= 1.0)
  {
    OptimalControlTrajectory::getOptimalT(start_point, end_point, maxVelocity, optimalT);
    T = optimalT;
  }
  else
  {
  dist_to_goal = sqrt((x_fc(0) - x_ic(0))*(x_fc(0) - x_ic(0)) + (y_fc(0) - y_ic(0))*(y_fc(0) - y_ic(0)) + (z_fc(0) - z_ic(0))*(z_fc(0) - z_ic(0)));
  T = 2*dist_to_goal/maxVelocity;
  optimalT = T;  
  }*/




  
  int n = 15; // points on the trajectory used in all the calculations including start and end point,
  VectorXd timePoints(n);

  /*double dist_to_goal;
  dist_to_goal = sqrt((x_fc(0) - x_ic(0))*(x_fc(0) - x_ic(0)) + (y_fc(0) - y_ic(0))*(y_fc(0) - y_ic(0)) + (z_fc(0) - z_ic(0))*(z_fc(0) - z_ic(0)));
  T = 2*dist_to_goal/maxVelocity;
  optimalT = T;*/

  // what if the initial speed is higher than the specified max speed, this should not happen but lets see what happens when we
  double initialSpeed = sqrt(start_point(1)*start_point(1)+start_point(5)*start_point(5)+start_point(9)*start_point(9));
  //maxVelocity = max(maxVelocity, initialSpeed);
  //cout <<"new trajectory."<<endl;
  

  while (regenerate_trajectory)
  {
    double _a, _b, _c, _d, _e;
    _a = 1.0/720; _b = 1.0/120; _c = 1.0/24; _d = 1.0/6; _e = 0.5;
    double delta = T / (n - 1);

    timePoints[0] = 0.0;
    for (int i = 0; i < (n - 1); ++i)
    {
      timePoints[i] =  delta * i;
    }
    timePoints[n-1] = T;

    minSnapCoeff(x_ic, x_fc, T, px); 
    minSnapCoeff(y_ic, y_fc, T, py); 
    minSnapCoeff(z_ic, z_fc, T, pz);

    VectorXd v(n), thrust(n), angularVelocity(n);

    
    for (int i = timePoints.size(); i--;)
    //for (int i = (n - 1); i--;)
    {
      double vx, vy, vz, ax, ay, az, jx, jy, jz, j;
      double t = timePoints[i];
      vx = px(0)*t*t*t*t*t*t*_a - px(1)*t*t*t*t*t*_b + px(2)*t*t*t*t*_c - px(3)*t*t*t*_d + px(4)*t*t*_e + px(5)* t + px(6);
      vy = py(0)*t*t*t*t*t*t*_a - py(1)*t*t*t*t*t*_b + py(2)*t*t*t*t*_c - py(3)*t*t*t*_d + py(4)*t*t*_e + py(5)* t + py(6);
      vz = pz(0)*t*t*t*t*t*t*_a - pz(1)*t*t*t*t*t*_b + pz(2)*t*t*t*t*_c - pz(3)*t*t*t*_d + pz(4)*t*t*_e + pz(5)* t + pz(6);

      ax = px(0)*t*t*t*t*t*_b - px(1)*t*t*t*t*_c + px(2)*t*t*t*_d - px(3)*t*t*_e + px(4)*t + px(5);
      ay = py(0)*t*t*t*t*t*_b - py(1)*t*t*t*t*_c + py(2)*t*t*t*_d - py(3)*t*t*_e + py(4)*t + py(5);
      az = pz(0)*t*t*t*t*t*_b - pz(1)*t*t*t*t*_c + pz(2)*t*t*t*_d - pz(3)*t*t*_e + pz(4)*t + pz(5);

      jx = px(0)*t*t*t*t*_c - px(1)*t*t*t*_d + px(2)*t*t*_e - px(3)*t + px(4);
      jy = py(0)*t*t*t*t*_c - py(1)*t*t*t*_d + py(2)*t*t*_e - py(3)*t + py(4);
      jz = pz(0)*t*t*t*t*_c - pz(1)*t*t*t*_d + pz(2)*t*t*_e - pz(3)*t + pz(4);

      //v(i) = sqrt(vx*vx + vy*vy + vz*vz);
      //j = sqrt(jx*jx + jy*jy + jz*jz);
      //thrust(i) = mass * sqrt(ax*ax + ay*ay + (az+9.8)*(az+9.8));
      //angularVelocity(i) = j/thrust(i);
      
      // these are square values to avoid sqrt function in the loop
      v(i) = vx*vx + vy*vy + vz*vz;
      j = jx*jx + jy*jy + jz*jz;
      thrust(i) = mass*mass * (ax*ax + ay*ay + (az+9.8)*(az+9.8));
      angularVelocity(i) = j/thrust(i);

    }
    double vel_limit = 5.0;
    if (sqrt(v.maxCoeff()) > vel_limit || sqrt(thrust.maxCoeff()) > maxThrust ||  sqrt(angularVelocity.maxCoeff()) > maxAngularVelocity)
    {
      if (T < optimalT * 1.4)
      {
        T = T * 1.1;
        regenerate_trajectory = true;
        //trajectory_is_feasible = false;
        limit_exceeded = true;
      }
      else
      {
        //cout<<"2:"<<T<<endl;
        //cout<< "operating point is close to inflection point, returning an infeasible trajectory, check boundary conditions."<< endl;
        regenerate_trajectory = false;
        //badTrajectory = true;
      }
    }
    else
    {
      regenerate_trajectory = false;
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





