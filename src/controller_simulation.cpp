#include<controller_simulation.h>

using namespace std;
using namespace Eigen;

Eigen::Vector4d controller_simulation::rot2quat(const Eigen::Matrix3d &R) 
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


Eigen::Matrix3d controller_simulation::quat2rot(const Eigen::Vector4d &q) 
{
  // Not needed for now
  Eigen::Matrix3d R;
  R << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return R;
}

void controller_simulation::calculateCommands(const nav_msgs::Odometry::ConstPtr& msg, VectorXd& desiredState, std::vector<double>& kx, 
                                  std::vector<double>& kv,std::vector<double>& kr, std::vector<double>& kw, double& mass, VectorXd& w) 
{
  //desired state, hard-coded for quick check, eventually be taken from the trajectoy_msg
  // it wont be final goal but a point on the trajectory
  Eigen::Vector3d Xc, Vc, Omegac, Xd, Vd, ad, kp, kd, kR, kOmega, ex, ev, _b1d, b1d, _b3d, b3d, b2d, eR;
  Eigen::Matrix3d Rd, _eR;
  Eigen::Vector4d qd;
  double normalized_thrust;
  Eigen::Vector3d moments, unbalanced_mass;
  Eigen::Vector3d Omegad, eOmega;
  Eigen::Vector4d T;
  Eigen::MatrixXd C(4,6);


  Xd << desiredState(0), desiredState(4), desiredState(8);
  Vd << desiredState(1), desiredState(5), desiredState(9);
  ad << desiredState(2), desiredState(6), desiredState(10);

  // current state
  Xc << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  Vc << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  Omegac << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

  Eigen::Quaterniond q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3d Rc = q.toRotationMatrix();


  // The velocity should be converted from bodyframe to inrtial, 
  // this is just for the simulation, it is not required otherwise
  Vc = Rc*Vc; 
  // remember to not use it when using openvins with actual hardware

  ex = Xc - Xd;
  ev = Vc - Vd;

  double yaw = desiredState(12);
  double yawrate = desiredState(13); 

  _b1d = {cos(yaw), sin(yaw), 0}; 

  kp << kx[0], kx[1], kx[2];
  kd << kv[0], kv[1], kv[2];

  _b3d = -(kp.cwiseProduct(ex) + kd.cwiseProduct(ev))/ mass + ad + g;
  b3d = _b3d.normalized();
  b2d = (b3d.cross(_b1d)).normalized();
  b1d = (b2d.cross(b3d)).normalized();

  Rd << b1d(0), b2d(0), b3d(0), b1d(1), b2d(1), b3d(1), b1d(2), b2d(2), b3d(2);


  Omegad << 0, 0, yawrate; //basically approximate controller, will have to take the derivative of Rd and find Omegad from that.
  // the derivative can be taken numerically or theoretically but I have noticed it wont change result much, not needed for px4 anyway
  // the logic is PD over eOmega takes care of minor differences.

  _eR = 0.5 * (Rd.transpose() * Rc - Rc.transpose() * Rd); 

  // matrix are populated column wise so _eR(7) is _eR(1,2)
  //eR << -_eR(7), -_eR(2), -_eR(3);
  eR << _eR(5), _eR(6), _eR(1);

  eOmega = Omegac - (Rc.transpose()*Rd)*Omegad;
  double thrust =  mass * _b3d.dot(Rd.col(2));

  kR << kr[0], kr[1], kr[2];
  kOmega << kw[0], kw[1], kw[2];



  moments = - kR.cwiseProduct(eR) - kOmega.cwiseProduct(eOmega) + Omegac.cross(J*Omegac);// - J*Z;
  unbalanced_mass<<0, -0.067, 0;
  moments = moments + unbalanced_mass;

  T << moments(0), moments(1), moments(2), thrust;
  C << sin(PI/6)*cg2cor*tau_t, sin(3*PI/6)*cg2cor*tau_t, sin(5*PI/6)*cg2cor*tau_t, sin(7*PI/6)*cg2cor*tau_t, sin(9*PI/6)*cg2cor*tau_t, sin(11*PI/6)*cg2cor*tau_t,
      -cos(PI/6)*cg2cor*tau_t, -cos(3*PI/6)*cg2cor*tau_t, -cos(5*PI/6)*cg2cor*tau_t, -cos(7*PI/6)*cg2cor*tau_t, -cos(9*PI/6)*cg2cor*tau_t, -cos(11*PI/6)*cg2cor*tau_t,
      -tau_t*tau_m, tau_t*tau_m, -tau_t*tau_m, tau_t*tau_m, -tau_t*tau_m, tau_t*tau_m,
      tau_t, tau_t, tau_t, tau_t, tau_t, tau_t;
  
  VectorXd w_square;

  Eigen::MatrixXd Cinv(6,4);
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(C);
  Cinv = cqr.pseudoInverse(); 
  //Cinv = C.completeOrthogonalDecomposition().pseudoInverse();
  w_square = Cinv*T;
  for (int i = 0; i < w_square.size(); i++)
  {
    w(i) = sqrt(abs(w_square(i)));
  }

  control_counter += 1;
  

}

