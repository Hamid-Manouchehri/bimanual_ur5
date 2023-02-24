#include <iostream>
#include <fstream>
#include <ctime>
#include "traj6d.h"
#include "traj6d.cpp"


int main(){

  traj_gen::quintic_traj traj;

  double t0, t1, t2, t_end, dt, wrist_3_length, obj_length, objCOMinWrist3_r,
  initPoseOfRightWrist3[3], finalPoseOfRightWrist3[3], offsetAlongLocalYaxis;
  Eigen::Vector3d w0, wf, dw0, dwf;
  Eigen::Vector3d p0, v0, a0, pf, vf, af;
  Eigen::Vector3d w, dw, p, v, a;
  Eigen::Quaterniond quat;
  std::vector<double> t_via, eulerToQuat0, eulerToQuatf, eulerToQuat1, eulerToQuat2;
  std::vector<Eigen::Quaterniond> quat_via;

  std::string dataFileName = "./traj_data/trajectory_data_bimanual.txt";  // TODO: uncomment for 'main_bimanual.py'.
  // std::string dataFileName = "./traj_data/trajectory_data_right.txt";  // TODO: uncomment for 'main_right_arm.py'.
  // std::string dataFileName = "./traj_data/trajectory_data_left.txt";  // TODO: uncomment for 'main_left_arm.py'.

  dt = 0.001;

  t0 = 0;
  t1 = 1.5;  // TODO: via time for 'quat1'
  t2 = 2;  // TODO: via time for 'quat2'
  t_end = 3;  // TODO: must be same as 't_end' of main simulation node.

  // t0 = 13.5;
  // t1 = 14.25;  // TODO: via time for 'quat1'
  // t2 = 14.5;  // TODO: via time for 'quat2'
  // t_end = 15;  // TODO: must be same as 't_end' of main simulation node.

  //// Note: generalized postion of object is defined according to 'wrist_3_link_r' link:
  wrist_3_length = 0.0823;  // based on 'ur5.urdf.xacro'
  obj_length = 0.2174;  // based on 'ur5.urdf.xacro'
  objCOMinWrist3_r = obj_length / 2 + wrist_3_length;  // length along 'wrist_3_link_r' y-axis

  offsetAlongLocalYaxis = objCOMinWrist3_r;  // TODO: uncommment when object is added to 'wrist_3_link_r', SINGLE and BIMANUAL scenarios.
  // offsetAlongLocalYaxis = 0;  // TODO: uncommment when object is removed from 'wrist_3_link_r', SINGLE right ur5

  // initialPoseOfWrist3_y = .191;  // TODO: uncomment for 'wrist_3_link_l', SINGLE left ur5
  // offsetAlongLocalYaxis = 0;  // TODO: uncommment for 'wrist_3_link_l', SINGLE left ur5




  //// initial linear position of 'wrist_3_link_r' frame in world frame:
  initPoseOfRightWrist3[0] = .552;  // x_i
  initPoseOfRightWrist3[1] = -.191;  // y_i
  initPoseOfRightWrist3[2] = .166;  // z_i
  //// final linear position of 'wrist_3_link_r' frame (x, y, z) in world frame:
  finalPoseOfRightWrist3[0] = initPoseOfRightWrist3[0];  // x_f
  finalPoseOfRightWrist3[1] = initPoseOfRightWrist3[1];  // y_f
  finalPoseOfRightWrist3[2] = initPoseOfRightWrist3[2];  // z_f

  //// initial linear position of 'wrist_3_link_r' frame in world frame:
  // initPoseOfRightWrist3[0] = .552;  // x_i
  // initPoseOfRightWrist3[1] = -.391;  // y_i
  // initPoseOfRightWrist3[2] = .266;  // z_i
  // //// final linear position of 'wrist_3_link_r' frame (x, y, z) in world frame:
  // finalPoseOfRightWrist3[0] = initPoseOfRightWrist3[0];  // x_f
  // finalPoseOfRightWrist3[1] = initPoseOfRightWrist3[1] + .2;  // y_f
  // finalPoseOfRightWrist3[2] = initPoseOfRightWrist3[2] - .1;  // z_f

  //// Note #1: the euler order is 'YZX' to change euler angles to quaternions.
  //// Note #2: object has 'pi' rad initial orientation about 'y-axis' in world frame.
  //// Note #3: for each simulation uncomment one set of orientations.
  //// [w, x, y, z]
  Eigen::Quaterniond quat0(0, 0, 1, 0);  // initial angular position, uncomment for 'wrist_3_link_r',  pi (rad) z-axis

  Eigen::Quaterniond quat1(0, 0, 1, 0);  // TODO: steady
  Eigen::Quaterniond quat2(0, 0, 1, 0);  // TODO: steady
  Eigen::Quaterniond quatf(0, 0, 1, 0);  // TODO: steady



  // Eigen::Quaterniond quat1(0, 0, .9848, -.1737);  // TODO: -20 (deg) x-axis
  // Eigen::Quaterniond quat2(0, 0, .9397, -0.342);  // TODO: -40 (deg) x-axis
  // Eigen::Quaterniond quatf(0, 0, .8661, -0.5);  // TODO: -60 (deg) x-axis

  //// vice versa:
  // Eigen::Quaterniond quat0(0, 0, .8661, -0.5);  // TODO: -60 (deg) x-axis
  // Eigen::Quaterniond quat1(0, 0, .9397, -0.342);  // TODO: -40 (deg) x-axis
  // Eigen::Quaterniond quat2(0, 0, .9848, -.1737);  // TODO: -20 (deg) x-axis
  // Eigen::Quaterniond quatf(0, 0, 1, 0);



  // Eigen::Quaterniond quat1(-0.174, 0, .9848, 0);  // TODO: pi (rad) + 20 (deg) y-axis
  // Eigen::Quaterniond quat2(-0.3420, 0, .9397, 0);  // TODO: pi (rad) + 40 (deg) y-axis
  // Eigen::Quaterniond quatf(-0.5, 0, .8660, 0);  // TODO: pi (rad) + pi/3 (rad) y-axis

  //// vice-versa:
  // Eigen::Quaterniond quat0(-0.5, 0, .8660, 0);  // TODO: pi (rad) + pi/3 (rad) y-axis
  // Eigen::Quaterniond quat1(-0.3420, 0, .9397, 0);  // TODO: pi (rad) + 40 (deg) y-axis
  // Eigen::Quaterniond quat2(-0.174, 0, .9848, 0);  // TODO: pi (rad) + 20 (deg) y-axis
  // Eigen::Quaterniond quatf(0, 0, 1, 0);



  // Eigen::Quaterniond quat1(0, -0.1737, .9848, 0.);  // TODO: 20 (deg) z-axis
  // Eigen::Quaterniond quat2(0, -0.3420, .9397, 0.);  // TODO: 40 (deg) z-axis
  // Eigen::Quaterniond quatf(0, -.5, .8661, 0.);  // TODO: 60 (deg) z-axis

  //// vice versa:
  // Eigen::Quaterniond quat0(0, -.5, .8661, 0.);  // TODO: 60 (deg) z-axis
  // Eigen::Quaterniond quat1(0, -0.3420, .9397, 0.);  // TODO: 40 (deg) z-axis
  // Eigen::Quaterniond quat2(0, -0.1737, .9848, 0.);  // TODO: 20 (deg) z-axis
  // Eigen::Quaterniond quatf(0, 0, 1, 0);



  // Eigen::Quaterniond quat1(-.171, -0.03, .9698, -.171);  // TODO: 20 (deg) xy-axis
  // Eigen::Quaterniond quat2(-0.3214, -0.117, .8830, -0.3214);  // TODO: 40 (deg) xy-axis
  // Eigen::Quaterniond quatf(-0.4330, -0.25, .75, -0.4329);  // TODO: 60 (deg) xy-axis

  //// vice versa:
  // Eigen::Quaterniond quat0(-0.4330, -0.25, .75, -0.4329);  // TODO: 60 (deg) xy-axis
  // Eigen::Quaterniond quat1(-0.3214, -0.117, .8830, -0.3214);  // TODO: 40 (deg) xy-axis
  // Eigen::Quaterniond quat2(-.171, -0.03, .9698, -.171);  // TODO: 20 (deg) xy-axis
  // Eigen::Quaterniond quatf(0, 0, 1, 0);

  // Eigen::Quaterniond quat1(-.03, .171, .9698, -.171);  // TODO: 20 (deg) xz-axis
  // Eigen::Quaterniond quat2(-0.117, 0.3214, .8830, -0.3214);  // TODO: 40 (deg) xz-axis
  // Eigen::Quaterniond quatf(-0.25, 0.433, .75, -0.433);  // TODO: 60 (deg) xz-axis

  // Eigen::Quaterniond quat1(-.171, .171, .9698, -.03);  // TODO: 20 (deg) yz-axis
  // Eigen::Quaterniond quat2(-0.3214, 0.3214, .8830, -0.117);  // TODO: 40 (deg) yz-axis
  // Eigen::Quaterniond quatf(-0.433, 0.433, .75, -0.25);  // TODO: 60 (deg) yz-axis



  //// SINGLE left ur5:
  // Eigen::Quaterniond quat0(0, 0, 0, 1);  // TODO: initial angular position, uncomment for 'wrist_3_link_l'(quaternion (w, q1, q2, q3)),  pi (rad) z-axis
  // Eigen::Quaterniond quat1(0, 0.2588, 0, .9659);  // TODO: pi/6 (rad) z-axis
  // Eigen::Quaterniond quat2(0, 0.3827, 0, .9239);  // TODO: pi/4 (rad) z-axis
  // Eigen::Quaterniond quatf(0, 0.4999, 0, .8661);  // TODO: pi/3




  p0 << initPoseOfRightWrist3[0], initPoseOfRightWrist3[1] + offsetAlongLocalYaxis, initPoseOfRightWrist3[2];
  pf << finalPoseOfRightWrist3[0], finalPoseOfRightWrist3[1] + offsetAlongLocalYaxis, finalPoseOfRightWrist3[2];

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
    myfile << quat.w();  // scalar part of quat
    myfile << " ";
    myfile << quat.vec().transpose();  // vector part of quat
    myfile << " ";
    myfile << w.transpose();
    myfile << " ";
    myfile << dw.transpose();
    myfile << "\n";

  }

  myfile.close();

  return 0;
}
