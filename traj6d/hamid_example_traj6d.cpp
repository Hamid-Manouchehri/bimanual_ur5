#include <iostream>
#include <fstream>
#include <ctime>
#include "traj6d.h"
#include "traj6d.cpp"


int main(){

  traj_gen::quintic_traj traj;

  double t0, t1, t2, t_end, dt, wrist_3_length, obj_length, objCOMinWrist3_r,
  initialPoseOfWrist3_y, offsetAlongLocalYaxis;
  Eigen::Vector3d w0, wf, dw0, dwf;
  Eigen::Vector3d p0, v0, a0, pf, vf, af;
  Eigen::Vector3d w, dw, p, v, a;
  Eigen::Quaterniond quat;
  std::vector<double> t_via, eulerToQuat0, eulerToQuatf, eulerToQuat1, eulerToQuat2;
  std::vector<Eigen::Quaterniond> quat_via;

  // std::string dataFileName = "trajectory_data_right.txt";  // TODO: uncomment for 'main_right_arm.py'.
  // std::string dataFileName = "trajectory_data_left.txt";  // TODO: uncomment for 'main_left_arm.py'.
  std::string dataFileName = "trajectory_data_bimanual.txt";  // TODO: uncomment for 'main_bimanual.py'.

  dt = 0.001;




  t0 = 0;
  t1 = 1.5;  // TODO: via time for 'quat1'
  t2 = 3;  // TODO: via time for 'quat2'
  t_end = 4;  // TODO: must be same as 't_end' of main simulation node.

  // Note: generalized postion of object is defined according to 'wrist_3_link_r' link:
  wrist_3_length = 0.0823;  // based on 'ur5.urdf.xacro'
  obj_length = 0.2174;  // based on 'ur5.urdf.xacro'
  objCOMinWrist3_r = obj_length / 2 + wrist_3_length;  // length along 'wrist_3_link_r' y-axis



  initialPoseOfWrist3_y = -.191;  // TODO: uncomment for 'wrist_3_link_r'
  offsetAlongLocalYaxis = objCOMinWrist3_r;  // TODO: uncommment when object is added to 'wrist_3_link_r', SINGLE and BIMANUAL scenarios.
  // offsetAlongLocalYaxis = 0;  // TODO: uncommment when object is removed from 'wrist_3_link_r', SINGLE right ur5

  // initialPoseOfWrist3_y = .191;  // TODO: uncomment for 'wrist_3_link_l', SINGLE left ur5
  // offsetAlongLocalYaxis = 0;  // TODO: uncommment for 'wrist_3_link_l', SINGLE left ur5

  p0 << .552, initialPoseOfWrist3_y + offsetAlongLocalYaxis, .166;  // initial linear position of object (x, y, z), according to 'wrist_3_link'
  pf << .552, initialPoseOfWrist3_y + offsetAlongLocalYaxis, .166;  // final linear position

  //// [w, x, y, z]
  Eigen::Quaterniond quat0(0, 0, 1, 0);  // TODO: initial angular position, uncomment for 'wrist_3_link_r',  pi (rad) z-axis
  // Eigen::Quaterniond quat1(0, 0, 1, 0);  // fixed
  // Eigen::Quaterniond quat2(0, 0, 1, 0);  // fixed
  // Eigen::Quaterniond quatf(0, 0, 1, 0);  // fixed

  // Eigen::Quaterniond quat1(0, 0, .9659, -.2588);  // TODO: pi + pi/6 (rad) x-axis
  // Eigen::Quaterniond quat2(0, 0, .9239, -0.3827);  // TODO: pi + pi/4 (rad) x-axis
  // Eigen::Quaterniond quatf(0, 0, .866, -0.5);  // TODO: pi + pi/3 (rad) x-axis

  Eigen::Quaterniond quat1(-0.2588, 0, .9659, 0);  // TODO: -pi + pi/6 (rad) y-axis
  Eigen::Quaterniond quat2(-0.3827, 0, .9239, 0);  // TODO: -pi + pi/4 (rad) y-axis
  Eigen::Quaterniond quatf(-0.5, 0, .866, 0);  // TODO: -pi + pi/3 (rad) y-axis

  // Eigen::Quaterniond quat1(0, -0.2588, .9659, 0.);  // TODO: pi + pi/6 (rad) z-axis
  // Eigen::Quaterniond quat2(0, -0.3827, .9239, 0.);  // TODO: pi + pi/4 (rad) z-axis
  // Eigen::Quaterniond quatf(0, -0.5, .866, 0.);  // TODO: pi + pi/3 (rad) z-axis







  // SINGLE left ur5:
  // Eigen::Quaterniond quat0(0, 0, 0, 1);  // TODO: initial angular position, uncomment for 'wrist_3_link_l'(quaternion (w, q1, q2, q3)),  pi (rad) z-axis
  // Eigen::Quaterniond quat1(0, 0.2588, 0, .9659);  // TODO: pi/6 (rad) z-axis
  // Eigen::Quaterniond quat2(0, 0.3827, 0, .9239);  // TODO: pi/4 (rad) z-axis
  // Eigen::Quaterniond quatf(0, 0.4999, 0, .8661);  // TODO: pi/3





  v0.setZero();  // initial linear velocity
  vf.setZero();  // final linear velocity
  a0.setZero();  // initial linear acceleration
  af.setZero();  // final linear acceleration

  w0.setZero();  // initial angular velocity (omega (roll, pitch, yaw))
  wf.setZero();  // final angular velocity (omega (roll, pitch, yaw))
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
