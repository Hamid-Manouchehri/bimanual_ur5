#include <iostream>
#include <fstream>
#include <ctime>
#include "traj6d.h"
#include "traj6d.cpp"


std::vector<double> ZYX_Euler_to_Quaternion(float phi, float theta, float psy){

  float w, q1, q2, q3;
  std::vector<double> output;

  w = cos(phi/2)*cos(theta/2)*cos(psy/2) + sin(phi/2)*sin(theta/2)*sin(psy/2);
  q1 = -cos(phi/2)*sin(theta/2)*sin(psy/2) + cos(theta/2)*cos(psy/2)*sin(phi/2);
  q2 = cos(phi/2)*cos(psy/2)*sin(theta/2) + sin(phi/2)*cos(theta/2)*sin(psy/2);
  q3 = cos(phi/2)*cos(theta/2)*sin(psy/2) - sin(phi/2)*cos(psy/2)*sin(theta/2);

  output.push_back(w);
  output.push_back(q1);
  output.push_back(q2);
  output.push_back(q3);

  return output;
}

int main(){

  traj_gen::quintic_traj traj;

  double t0, t1, t2, t_end, dt, wrist_3_length, obj_length, objCOMinWrist3;
  Eigen::Vector3d w0, wf, dw0, dwf;
  Eigen::Vector3d p0, v0, a0, pf, vf, af;
  Eigen::Vector3d w, dw, p, v, a;
  Eigen::Quaterniond quat;
  std::vector<double> t_via, eulerToQuat0, eulerToQuatf, eulerToQuat1, eulerToQuat2;
  std::vector<Eigen::Quaterniond> quat_via;
  std::string dataFileName = "trajectory_data.txt";

  dt = 0.001;




  t0 = 0;
  t1 = 1.5;  // TODO: for 'quat1'
  t2 = 3;  // TODO: for 'quat2'
  t_end = 4.5;  // TODO: must be same as 't_end' of main simulation node.

  wrist_3_length = 0.0823;  // based on 'ur5.urdf.xacro'
  obj_length = 0.2174;  // based on 'ur5.urdf.xacro'
  objCOMinWrist3 = obj_length / 4 + wrist_3_length;  // length along 'wrist_3_link' y-axis



  // p0 << .3922, .1092, .609;  // TODO: initial linear position (x, y, z), uncomment as object is removed
  p0 << .3922, -.191 + objCOMinWrist3, .609;  // TODO: initial linear position (x, y, z), uncomment as object is added
  pf << .3922, -.191 + objCOMinWrist3, .609;  // final linear position

  v0.setZero();  // initial linear velocity
  vf.setZero();  // final linear velocity
  a0.setZero();  // initial linear acceleration
  af.setZero();  // final linear acceleration

  eulerToQuat0 = ZYX_Euler_to_Quaternion(0, 0, 0);
  eulerToQuat1 = ZYX_Euler_to_Quaternion(0, 0, M_PI/6);
  eulerToQuat2 = ZYX_Euler_to_Quaternion(0, 0, M_PI/3);
  eulerToQuatf = ZYX_Euler_to_Quaternion(0, 0, M_PI/2);



  // Eigen::Quaterniond quat0(eulerToQuat0[0], eulerToQuat0[1], eulerToQuat0[2], eulerToQuat0[3]);  // initial angular position (quaternion (w, q1, q2, q3)),  0 (rad) z-axis
  // Eigen::Quaterniond quat1(eulerToQuat1[0], eulerToQuat1[1], eulerToQuat1[2], eulerToQuat1[3]);  // (w, q1, q2, q3),  pi/6 (rad) z-axis
  // Eigen::Quaterniond quat2(eulerToQuat2[0], eulerToQuat2[1], eulerToQuat2[2], eulerToQuat2[3]);  // pi/3 (rad) z-axis
  // Eigen::Quaterniond quatf(eulerToQuatf[0], eulerToQuatf[1], eulerToQuatf[2], eulerToQuatf[3]);  // final angular position (quaternion),   pi/2 (rad) z-axis

  Eigen::Quaterniond quat0(1, 0, 0, 0);  // initial angular position (quaternion (w, q1, q2, q3)),  0 (rad) z-axis
  Eigen::Quaterniond quat1(.9659, 0, 0, .2588);  // (w, q1, q2, q3),  pi/6 (rad) z-axis
  Eigen::Quaterniond quat2(.9239, 0, 0, .3827);  // pi/4 (rad) z-axis
  Eigen::Quaterniond quatf(.8661, 0, 0, .4999);  // pi/3
  // Eigen::Quaterniond quat1(1, 0, 0, 0);  // (w, q1, q2, q3),  pi/6 (rad) z-axis
  // Eigen::Quaterniond quat2(1, 0, 0, 0);  // pi/4 (rad) z-axis
  // Eigen::Quaterniond quatf(1, 0, 0, 0);  // pi/3




  w0.setZero();  // initial angular velocity (omega (roll, pitch, yaw))
  wf.setZero(); // final angular velocity (omega (roll, pitch, yaw))
  dw0.setZero();  // initial angular acceleration (alpha (roll, pitch, yaw))
  dwf.setZero();  // final angular acceleration (alpha (roll, pitch, yaw))






  t_via.push_back(t1);
  t_via.push_back(t2);
  quat_via.push_back(quat1);
  quat_via.push_back(quat2);

  traj.setInitialTime(t0);
  traj.setFinalTime(t_end);
  // initial linear states:
  traj.setInitialPosition_Linear(p0);
  traj.setInitialVelocity_Linear(v0);
  traj.setInitialAcceleration_Linear(a0);
  // final linear states:
  traj.setFinalPosition_Linear(pf);
  traj.setFinalVelocity_Linear(vf);
  traj.setFinalAcceleration_Linear(af);
  // initial angular states:
  traj.setInitialPosition_Angular(quat0);
  traj.setInitialVelocity_Angular(w0);
  traj.setInitialAcceleration_Angular(dw0);
  // final angular states:
  traj.setFinalPosition_Angular(quatf);
  traj.setFinalVelocity_Angular(wf);
  traj.setFinalAcceleration_Angular(dwf);
  // define via points
  traj.setViaPointsTime(t_via);
  traj.setViaPointsPosition_Angular(quat_via);

  traj.generateTraj_Linear("quintic");
  traj.generateTraj_Angular("spline_poly", true, dt);  // TBD seems only works with dt = 0.001

  std::ofstream myfile;
  myfile.open (dataFileName);  // create a file to store trajectory data.

  // store data to the file:
  for (double t=t0; t<=t_end; t+=dt){

    traj.getState_Linear(t, p, v, a);  // read linear states in each iteration
    traj.getState_Angular(t, quat, w, dw);  // read angular states in each iteration

    // save data to 'dataFileName':
    myfile << t;
    myfile << " ";
    myfile << p.transpose();
    myfile << " ";
    myfile << v.transpose();
    myfile << " ";
    myfile << a.transpose();
    myfile << " ";
    myfile << quat.w();
    myfile << " ";
    myfile << quat.vec().transpose();
    myfile << " ";
    myfile << w.transpose();
    myfile << " ";
    myfile << dw.transpose();
    myfile << "\n";

  }

  myfile.close();

  return 0;
}
