#include "traj6d.h"
#include <iostream>
#include <ctime>


namespace traj_gen{

quintic_traj::quintic_traj(){

//  dt = time_step;

 t0 = 0;
 tf = 1;
//  t_now = t0 + dt; //TBD
 t0_updated = t0;

 x0.setZero();
 dx0.setZero();
 ddx0.setZero();
 xf.setZero();
 dxf.setZero();
 ddxf.setZero();

 quat0.setIdentity();
 w0.setZero();
 dw0.setZero();
 quatf.setIdentity();
 wf.setZero();
 dwf.setZero();

 quat_now.setIdentity(); //TBD
 dquat_now.setIdentity(); //TBD
 ddquat_now.setIdentity(); //TBD

quat_old = quat0;
w_old = w0;
dw_old = dw0;

 quat_nan.coeffs() << NAN, NAN, NAN, NAN;

 ddN = 0;

}

void quintic_traj::setInitialPosition_Linear(const Eigen::Vector3d& init_pose){x0 = init_pose;}
void quintic_traj::setInitialVelocity_Linear(const Eigen::Vector3d& init_vel){dx0 = init_vel;}
void quintic_traj::setInitialAcceleration_Linear(const Eigen::Vector3d& init_acc){ddx0 = init_acc;}

void quintic_traj::setInitialPosition_Angular(const Eigen::Quaterniond& init_quat){quat0 = init_quat;}
void quintic_traj::setInitialVelocity_Angular(const Eigen::Vector3d& init_anlgular_vel){w0 = init_anlgular_vel;}
void quintic_traj::setInitialAcceleration_Angular(const Eigen::Vector3d& init_angular_acc){dw0 = init_angular_acc;}

void quintic_traj::setInitialTime(const double& init_time){t0 = init_time;}

void quintic_traj::setFinalPosition_Linear(const Eigen::Vector3d& final_pos){xf = final_pos;}
void quintic_traj::setFinalVelocity_Linear(const Eigen::Vector3d& final_vel){dxf = final_vel;}
void quintic_traj::setFinalAcceleration_Linear(const Eigen::Vector3d& final_acc){ddxf = final_acc;}

void quintic_traj::setFinalPosition_Angular(const Eigen::Quaterniond& final_quat){quatf = final_quat;}
void quintic_traj::setFinalVelocity_Angular(const Eigen::Vector3d& final_angular_vel){wf = final_angular_vel;}
void quintic_traj::setFinalAcceleration_Angular(const Eigen::Vector3d& final_angular_acc){dwf = final_angular_acc;}

void quintic_traj::setFinalTime(const double& final_time){tf = final_time;}

void quintic_traj::setViaPointsTime(const std::vector<double>& viapoints_time){t_viapts = viapoints_time;}
void quintic_traj::setViaPointsPosition_Angular(const std::vector<Eigen::Quaterniond>& viapoints_quat){quat_viapts = viapoints_quat;}

void quintic_traj::generateTraj_Linear(const std::string& type){
  traj_type_Linear = type;
 if (traj_type_Linear != "quintic" && traj_type_Linear != "cubic"){
   std::cout << "invalid type was passed" << std::endl; //TBD throw error
 }
  quintic_traj::computeCoeffs_Linear ();
}

void quintic_traj::generateTraj_Angular(const std::string& type, const bool method_successive,
					const double time_step){
  dt = time_step;
  traj_type_Angular = type;

  if (traj_type_Angular != "quintic" &&
      traj_type_Angular != "cubic" &&
      traj_type_Angular != "slerp" &&
      traj_type_Angular != "squad" &&
      traj_type_Angular != "spline_squad" &&
      traj_type_Angular != "spline_poly"
     )
  {
   std::cout << "invalid type was passed" << std::endl; //TBD throw error
  }

    double dot = quat0.dot(quatf);

    if (dot < 0.0f){
      std::cout << "dot product of quaternions is negative, I consider quatf*(-1)" << std::endl;
      quatf.w() *= -1;
      quatf.vec() *= -1;
    }

    if (traj_type_Angular == "slerp"){
      quintic_traj::computeQuaternionLog(quat0.inverse()*quatf, Logq0invq1);
    }
    else if (traj_type_Angular == "squad"){
      quintic_traj::computeSquadICs(quat0, quatf, w0, wf, tf - t0, squad_a, squad_b);
//       quintic_traj::computeQuaetrnionLog(quat0.inverse()*quatf, Logq0invq1);
    }
    else if(traj_type_Angular == "spline_squad"){
    squad_param_current.resize(4);
    quintic_traj::computeSquadSplineControlPoints();
    }
    else if(traj_type_Angular == "spline_poly"){

      //// equ(6,7): orientation planning:
      quintic_traj::w2dq(quat0, w0, dw0, dquat0, ddquat0);
      quintic_traj::w2dq(quatf, wf, dwf, dquatf, ddquatf);

      t_all = t_viapts;
      t_all.push_back(tf);
      t_all.insert(t_all.begin(), t0);

      quat_all = quat_viapts;
      quat_all.push_back(quatf);
      quat_all.insert(quat_all.begin(), quat0);


      N = t_all.size();

      ind_poly = 0;

      quat_elements_temp.resize(N);
      v_all.resize(N);
      abcd.a.resize(N - 2);
      abcd.b.resize(N - 2);
      abcd.c.resize(N - 2);
      abcd.d.resize(N - 2);
      cp.resize(N - 2);
      dp.resize(N - 2);
      v_temp.resize(N - 2);
      VQcoeffs.resize(N - 1);
      Vcoeffs.resize(N - 1);

      quat_old = quat0;
      w_old = w0;
      dw_old = dw0;


      dqddq = (dQddQ){dquat0, dquatf, ddquat0, ddquatf};
      quintic_traj::computePolySplineCpoints(t_all, quat_all, "343");

      qcoeffs = VQcoeffs[0];
      qcoeffs_old = qcoeffs;

      traj_Angular_successive = method_successive;

    }
    else {

      p1f_eigen = Eigen::Quaterniond(0, wf[0]/2, wf[1]/2, wf[2]/2);
      p2f_eigen = Eigen::Quaterniond(-.25*pow(wf.norm(), 2), dwf[0]/2, dwf[1]/2, dwf[2]/2);

      dquatf = p1f_eigen * quatf;  // equ(6)
      ddquatf = p2f_eigen * quatf;  // equ(7)

      traj_Angular_successive = method_successive;

      t0_updated = t0;
      quat_old = quat0;
      w_old = w0;
      dw_old = dw0;

      p_Angular.resize(6); // the last two elements can be left idle for cubic traj

      quintic_traj::computeCoeffs_Angular (true);
    }

}


void quintic_traj::w2dq(const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d& dw,
                        Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq)
{ //// equ(6,7): orientation planning
  p1_eigen.vec().noalias() = w/2;
  p1_eigen.w() = 0;
  p2_eigen.vec().noalias() = dw/2;
  p2_eigen.w() = -.25*pow(w.norm(), 2);
  dq = p1_eigen * q;
  ddq = p2_eigen * q;
}

void quintic_traj::getState_Linear(const double t, Eigen::Vector3d& x, Eigen::Vector3d& dx, Eigen::Vector3d& ddx){
//   t_now = t;

  x.noalias() = p_Linear.col(0) + p_Linear.col(1)*t + p_Linear.col(2)*pow(t, 2)
    + p_Linear.col(3)*pow(t, 3) + p_Linear.col(4)*pow(t, 4)
    + p_Linear.col(5)*pow(t, 5);

  dx.noalias() = p_Linear.col(1) + 2*p_Linear.col(2)*t
    + 3*p_Linear.col(3)*pow(t, 2) + 4*p_Linear.col(4)*pow(t, 3)
    + 5*p_Linear.col(5)*pow(t, 4);

  ddx.noalias() = 2*p_Linear.col(2)
    + 6*p_Linear.col(3)*t + 12*p_Linear.col(4)*pow(t, 2)
    + 20*p_Linear.col(5)*pow(t, 3);
}

/*
void quintic_traj::getPosition_Linear(const double t, Eigen::Vector3d& x){
  t_now = t;

  x = p_Linear.col(0) + p_Linear.col(1)*t + p_Linear.col(2)*pow(t, 2)
    + p_Linear.col(3)*pow(t, 3) + p_Linear.col(4)*pow(t, 4)
    + p_Linear.col(5)*pow(t, 5);

}

void quintic_traj::getVelocity_Linear(const double t, Eigen::Vector3d& dx){
  t_now = t;

  dx = p_Linear.col(1) + 2*p_Linear.col(2)*t
    + 3*p_Linear.col(3)*pow(t, 2) + 4*p_Linear.col(4)*pow(t, 3)
    + 5*p_Linear.col(5)*pow(t, 4);
}

void quintic_traj::getAcceleration_Linear(const double t, Eigen::Vector3d& ddx){
  t_now = t;

  ddx = 2*p_Linear.col(2)
    + 6*p_Linear.col(3)*t + 12*p_Linear.col(4)*pow(t, 2)
    + 20*p_Linear.col(5)*pow(t, 3);
}*/


void quintic_traj::computeQuinticQuatAndDerives(const std::vector<Eigen::Quaterniond>& p, const double t,
						Eigen::Quaterniond& quat,
						Eigen::Quaterniond& dquat,
						Eigen::Quaterniond& ddquat){
    quat.w() = p[0].w() + p[1].w()*t + p[2].w()*pow(t, 2)
      + p[3].w()*pow(t, 3) + p[4].w()*pow(t, 4)
      + p[5].w()*pow(t, 5);

    quat.vec() = p[0].vec() + p[1].vec()*t + p[2].vec()*pow(t, 2)
      + p[3].vec()*pow(t, 3) + p[4].vec()*pow(t, 4)
      + p[5].vec()*pow(t, 5);

    dquat.w() = p[1].w() + 2*p[2].w()*t
      + 3*p[3].w()*pow(t, 2) + 4*p[4].w()*pow(t, 3)
      + 5*p[5].w()*pow(t, 4);

    dquat.vec() = p[1].vec() + 2*p[2].vec()*t
      + 3*p[3].vec()*pow(t, 2) + 4*p[4].vec()*pow(t, 3)
      + 5*p[5].vec()*pow(t, 4);

    ddquat.w() = 2*p[2].w()
      + 6*p[3].w()*t + 12*p[4].w()*pow(t, 2)
      + 20*p[5].w()*pow(t, 3);

    ddquat.vec() = 2*p[2].vec()
      + 6*p[3].vec()*t + 12*p[4].vec()*pow(t, 2)
      + 20*p[5].vec()*pow(t, 3);


}


void quintic_traj::computeQuinticDoubleAndDerives(const std::vector<double>& p, const double t,
						  double x,
						  double dx,
						  double ddx){
    x = p[0] + p[1]*t + p[2]*pow(t, 2)
      + p[3]*pow(t, 3) + p[4]*pow(t, 4)
      + p[5]*pow(t, 5);

    dx = p[1] + 2*p[2]*t
      + 3*p[3]*pow(t, 2) + 4*p[4]*pow(t, 3)
      + 5*p[5]*pow(t, 4);

    ddx = 2*p[2]
      + 6*p[3]*t + 12*p[4]*pow(t, 2)
      + 20*p[5]*pow(t, 3);

}


// void quintic_traj::computeQuaternionAndDerives(){
//
//     Eigen::Quaterniond quat_here, dquat_here, ddquat_here;
//     quintic_traj::computeQuinticQuatAndDerives(p_Angular, t_now, quat_here, dquat_here, ddquat_here);
//
//     double N = quat_here.norm();
//
//     double DNperN, DDNperN;
//     quintic_traj::computeNDeriveParam(p_Angular, t_now, DNperN, DDNperN);
//
//     quat_now.w() = 1/N*quat_here.w();
//     quat_now.vec() = 1/N*quat_here.vec();
//
//     dquat_now.w() = 1/N*(dquat_here.w() - DNperN*quat_here.w());
//     dquat_now.vec() = 1/N*(dquat_here.vec() - DNperN*quat_here.vec());
//
//     ddquat_now.w() = 1/N*(ddquat_here.w() - 2*DNperN*dquat_here.w() + (DDNperN - 2*DNperN*DNperN)*quat_here.w());
//     ddquat_now.vec() = 1/N*(ddquat_here.vec() - 2*DNperN*dquat_here.vec() + (DDNperN - 2*DNperN*DNperN)*quat_here.vec());
// }

void quintic_traj::computeQuaternionAndDerives(const double t, Eigen::Quaterniond& q,
							       Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq){

  if (traj_type_Angular == "quintic"){

      q.coeffs().noalias() = pow(1 - t, 3)*(p_Angular[0].coeffs() + p_Angular[1].coeffs()*t + p_Angular[2].coeffs()*t*t) +
	    pow(t, 3)*(p_Angular[3].coeffs() + p_Angular[4].coeffs()*(1 - t) + p_Angular[5].coeffs()*(1 - t)*(1 - t));

      dq.coeffs().noalias() = pow(1 - t, 2)*(-3*p_Angular[0].coeffs() + p_Angular[1].coeffs()*(1 - 4*t) + p_Angular[2].coeffs()*(2*t - 5*t*t)) +
	    pow(t, 2)*(3*p_Angular[3].coeffs() + p_Angular[4].coeffs()*(3 - 4*t) + p_Angular[5].coeffs()*(3 - 8*t + 5*t*t));

      ddq.coeffs().noalias() = (1 - t)*(6*p_Angular[0].coeffs() - p_Angular[1].coeffs()*6*(1 - 2*t) + p_Angular[2].coeffs()*2*(1 - 8*t + 10*t*t)) +
	    t*(6*p_Angular[3].coeffs() + p_Angular[4].coeffs()*6*(1 - 2*t) + p_Angular[5].coeffs()*2*(3 - 12*t + 10*t*t));

  } else{
    Eigen::Quaterniond temp1, temp2;
    temp1.w() = p_Angular[1].w() - p_Angular[0].w();
    temp1.vec() = p_Angular[1].vec() - p_Angular[0].vec();
    temp2.w() = p_Angular[2].w()*t*t*t + p_Angular[3].w()*t*t + p_Angular[4].w()*t;
    temp2.vec() = p_Angular[2].vec()*t*t*t + p_Angular[3].vec()*t*t + p_Angular[4].vec()*t;
    temp2 = temp1*temp2;

    q.w() = p_Angular[0].w() + temp2.w();
    q.vec() = p_Angular[0].vec() + temp2.vec();

    temp2.w() = 3*p_Angular[2].w()*t*t + 2*p_Angular[3].w()*t + p_Angular[4].w();
    temp2.vec() = 3*p_Angular[2].vec()*t*t + 2*p_Angular[3].vec()*t + p_Angular[4].vec();

    dq = temp1*temp2;

    temp2.w() = 6*p_Angular[2].w()*t + 2*p_Angular[3].w();
    temp2.vec() = 6*p_Angular[2].vec()*t + 2*p_Angular[3].vec();

    ddq = temp1*temp2;
  }
}

void quintic_traj::getState_Angular(const double t, Eigen::Quaterniond& quat, Eigen::Vector3d& w,
						      Eigen::Vector3d& dw, const double period){
  dt = period;

  if (traj_type_Angular == "slerp"){
    double tau = (t - t0)/(tf - t0);
    double term1, term2, term3;

    quintic_traj::transformPhase2Time(tau, term1, term2, term3);
    quintic_traj::computeSlerp(term1, q, q_temp2, q_temp3);

    quat = q;
    dq.coeffs().noalias() = q_temp2.coeffs()*term2/(tf - t0);
    ddq.coeffs().noalias() = (q_temp3.coeffs()*term2 + q_temp2.coeffs()*term3)/pow(tf - t0, 2);


  }else if (traj_type_Angular == "squad")
  {
    double tau = (t - t0)/(tf - t0);

    quintic_traj::computeSquad(tau, q, dq, ddq);

    quintic_traj::computeDSquad_num(tau, quat0, quatf,
				    squad_a, squad_b, dq);
    dq.coeffs() /= tf - t0;
    quat = q;

  }else if (traj_type_Angular == "spline_squad")
  {
   double t0_here, T;
   quintic_traj::getSquadSplineCurrentParams(t, squad_param_current, t0_here, T);

//    std::cout << squad_param_current[1].coeffs().transpose()<< std::endl;

   double tau = (t - t0_here)/T;

   quintic_traj::computeSquadAux(tau, squad_param_current[0], squad_param_current[1],
				    squad_param_current[2], squad_param_current[3], q);

   quat = q;

   quintic_traj::computeDSquad_num(tau, squad_param_current[0], squad_param_current[1],
				    squad_param_current[2], squad_param_current[3], dq);

   dq.coeffs() /= T;
   ddq.coeffs() = quat.coeffs()*0;   //TBD: now uses fake value
  }
  else if (traj_type_Angular == "spline_poly")
  {
   double t0_here = t0;
   if (fabs(t - t0) > dt/2){
     // std::cout << "1" << std::endl;  // second jump here.
    quintic_traj::getPolySplineCurrentParams(t - dt, quat_old, w_old, dw_old, qcoeffs, t0_here);
  }

   if (std::isnan(qcoeffs.p4.w())){
     // std::cout << "2" << std::endl;
     quintic_traj::computeCubicTraj(qcoeffs, t - t0_here, q, dq, ddq);
   }

   else{
     // std::cout << "3" << std::endl;  // first jump here.
     quintic_traj::computeQuarticTraj(qcoeffs, t - t0_here, q, dq, ddq);
   }
   quat = q.normalized();

   qcoeffs_old = qcoeffs;

  }
  else{

    if (traj_Angular_successive && fabs(t - t0) > dt/2){
      t0_updated  = t - dt;
      quintic_traj::computeCoeffs_Angular();
    }

    double T = tf - t0_updated;
    double tau = (t - t0_updated)/T;

    quintic_traj::computeQuaternionAndDerives(tau, q, dq, ddq);  // equ(12) and derivatives
    quat = q.normalized();
    dq.coeffs().noalias() = dq.coeffs()/T;
    ddq.coeffs().noalias() = ddq.coeffs()/T/T;

  }

  q_inverse = q.inverse();

  w_quat = dq * q_inverse;
  w_quat.coeffs() *= 2;  // equ(6)
  w = w_quat.vec();

  dw.noalias() = (ddq * q_inverse).vec()*2 - (w_quat * dq * q_inverse).vec();  // equ(7)

  //save data for the next step:
  quat_old = quat;
  w_old = w;
  dw_old = dw;

}


void quintic_traj::computeReferenceAcceleration(const double t, const Eigen::Affine3d& actual_pose,
				    const Eigen::VectorXd& actual_vel,
				    Eigen::VectorXd& reference_acc){

  reference_acc.resize(6);

  kp = 3000;
  kd = kp/4;
  ko = 4000;
  ka = ko/4;

  quintic_traj::getState_Linear(t, x_des, dx_des, ddx_des);

  quintic_traj::getState_Angular(t, quat_des, w_des, dw_des);

  quat = Eigen::Quaterniond(actual_pose.linear());

//   quat_des = quat;
//   w_des.setZero();
//   dw_des.setZero();

//   std::cout << quat_des.w()*quat.vec() - quat.w()*quat_des.vec() + quat_des.vec().cross(quat.vec())  << std::endl;

  ddp_ref = ddx_des + kp*(x_des - actual_pose.translation()) + kd*(dx_des - actual_vel.head(3));
  dw_ref = dw_des + ka*(w_des - actual_vel.tail(3)) - ko *(
    quat_des.w()*quat.vec() - quat.w()*quat_des.vec() + quat_des.vec().cross(quat.vec())
  );
  reference_acc << ddp_ref , dw_ref;

}

void quintic_traj::computeReferenceError(const double t, const Eigen::Affine3d& actual_pose,
				    const Eigen::VectorXd& actual_vel,
				    Eigen::VectorXd& reference_err,
				    Eigen::VectorXd& reference_derr
					){

  reference_err.resize(6);
  reference_derr.resize(6);

  kp = 3000;
  kd = kp/4;
  ko = 4000;
  ka = ko/4;

  quintic_traj::getState_Linear(t, x_des, dx_des, ddx_des);

  quintic_traj::getState_Angular(t, quat_des, w_des, dw_des);

  quat = Eigen::Quaterniond(actual_pose.linear());

//   quat_des = quat;
//   w_des.setZero();
//   dw_des.setZero();

//   std::cout << quat_des.w()*quat.vec() - quat.w()*quat_des.vec() + quat_des.vec().cross(quat.vec())  << std::endl;

  reference_err << x_des - actual_pose.translation(),
  - 1. *(
    quat_des.w()*quat.vec() - quat.w()*quat_des.vec() + quat_des.vec().cross(quat.vec()));

  reference_derr << dx_des - actual_vel.head(3), w_des - actual_vel.tail(3);

//   ddp_ref = ddx_des + kp*(x_des - actual_pose.translation()) + kd*(dx_des - actual_vel.head(3));
//   dw_ref = dw_des + ka*(w_des - actual_vel.tail(3)) - ko *(
//     quat_des.w()*quat.vec() - quat.w()*quat_des.vec() + quat_des.vec().cross(quat.vec())
//   );
//   reference_acc << ddp_ref , dw_ref;

}

void quintic_traj::Quaternion2Euler(const Eigen::Quaterniond& quat, Eigen::Vector3d& euler){
  euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
}

void quintic_traj::Euler2Quaternion(const Eigen::Vector3d& euler, Eigen::Quaterniond& quat){
  Eigen::AngleAxisd rollAngle(euler[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(euler[2], Eigen::Vector3d::UnitZ());

    quat = rollAngle * pitchAngle * yawAngle;
}

void quintic_traj::getInitialPosition_Linear(Eigen::Vector3d& x){x = x0;}
void quintic_traj::getInitialVelocity_Linear(Eigen::Vector3d& dx){dx = dx0;}
void quintic_traj::getInitialAcceleration_Linear(Eigen::Vector3d& ddx){ddx = ddx0;}

void quintic_traj::getInitialPosition_Angular(Eigen::Quaterniond& quat){quat = quat0;}
void quintic_traj::getInitialVelocity_Angular(Eigen::Vector3d& w){w = w0;}
void quintic_traj::getInitialAcceleration_Angular(Eigen::Vector3d& dw){dw = dw0;}

void quintic_traj::getFinalPosition_Linear(Eigen::Vector3d& x){x = xf;}
void quintic_traj::getFinalVelocity_Linear(Eigen::Vector3d& dx){dx = dxf;}
void quintic_traj::getFinalAcceleration_Linear(Eigen::Vector3d& ddx){ddx = ddxf;}

void quintic_traj::getFinalPosition_Angular(Eigen::Quaterniond& quat){quat = quatf;}
void quintic_traj::getFinalVelocity_Angular(Eigen::Vector3d& w){w = wf;}
void quintic_traj::getFinalAcceleration_Angular(Eigen::Vector3d& dw){dw = dwf;}

void quintic_traj::Rotation_X(const double angle, Eigen::Matrix3d& rot_x){

  rot_x << 1.0, 0.0, 0.0,
           0.0, std::cos(angle),- std::sin(angle),
	   0.0, std::sin(angle), std::cos(angle);
}

void quintic_traj::Rotation_Y(const double angle, Eigen::Matrix3d& rot_y){

  rot_y << std::cos(angle), 0.0,  std::sin(angle),
           0.0, 1.0, 0.0,
	   -std::sin(angle), 0.0, std::cos(angle);
}

void quintic_traj::Rotation_Z(const double angle, Eigen::Matrix3d& rot_z){

  rot_z << std::cos(angle), - std::sin(angle), 0.0,
           std::sin(angle), std::cos(angle), 0.0,
	   0.0, 0.0, 1.0;
}



void quintic_traj::computeNDeriveParam(const std::vector<Eigen::Quaterniond>& p, const double t, double& DNperN, double& DDNperN){

  std::vector<double> a_all, b_all, c_all, d_all;
  double x, dx, ddx;
  Eigen::Vector4d q, dq, ddq;
  q.setZero();
  dq.setZero();
  ddq.setZero();

  for (int i=0;i<6;i++){
   a_all.push_back(p[i].w());
   b_all.push_back(p[i].x());
   c_all.push_back(p[i].y());
   d_all.push_back(p[i].z());
  }

  quintic_traj::computeQuinticDoubleAndDerives(a_all, t_now, x, dx, ddx);
  q[0] = x; dq[0] = dx; ddq[0] = ddx;

  quintic_traj::computeQuinticDoubleAndDerives(b_all, t_now, x, dx, ddx);
  q[1] = x; dq[1] = dx; ddq[1] = ddx;

  quintic_traj::computeQuinticDoubleAndDerives(c_all, t_now, x, dx, ddx);
  q[2] = x; dq[2] = dx; ddq[2] = ddx;

  quintic_traj::computeQuinticDoubleAndDerives(d_all, t_now, x, dx, ddx);
  q[3] = x; dq[3] = dx; ddq[3] = ddx;

  Eigen::Quaterniond quat, dquat, ddquat;
//   Eigen::Vector3d dummy;
  quintic_traj::computeQuinticQuatAndDerives(p, t_now, quat, dquat, ddquat);
  double N = quat.norm();

  DNperN = 1/N/N * q.dot(dq);
  DDNperN = 1/N/N * (q.dot(ddq) + dq.dot(dq)) - DNperN*DNperN;

}


void quintic_traj::computeDDN(const double t, const double T, double& ddN){

  Eigen::Quaterniond q, dq, ddq;
  quintic_traj::computeQuaternionAndDerives(t, q, dq, ddq);
  dq.coeffs() /= T;
  ddq.coeffs() /= pow(T, 2);

  double N = q.norm();
  double dN = 1/N*q.coeffs().dot(dq.coeffs());  // equ(17)
  ddN = 1/N*(q.coeffs().dot(ddq.coeffs()) + dq.coeffs().dot(dq.coeffs()) - dN*dN);  // equ(18)

}


void quintic_traj::computeStateForTest(const std::vector<Eigen::Quaterniond>& p, const double t,
				       Eigen::Quaterniond& quat_t,
				       Eigen::Vector3d& w_t,
				       Eigen::Vector3d& dw_t){

    Eigen::Quaterniond quat_here, dquat_here, ddquat_here;
    quintic_traj::computeQuinticQuatAndDerives(p, t, quat_here, dquat_here, ddquat_here);

    double N = quat_here.norm();

    double DNperN, DDNperN;
    quintic_traj::computeNDeriveParam(p, t, DNperN, DDNperN);

    quat_t.w() = 1/N*quat_here.w();
    quat_t.vec() = 1/N*quat_here.vec();

    Eigen::Quaterniond dquat_t, ddquat_t;
    dquat_t.w() = 1/N*(dquat_here.w() - DNperN*quat_here.w());
    dquat_t.vec() = 1/N*(dquat_here.vec() - DNperN*quat_here.vec());

    ddquat_t.w() = 1/N*(ddquat_here.w() - 2*DNperN*dquat_here.w() + (DDNperN - 2*DNperN*DNperN)*quat_here.w());
    ddquat_t.vec() = 1/N*(ddquat_here.vec() - 2*DNperN*dquat_here.vec() + (DDNperN - 2*DNperN*DNperN)*quat_here.vec());

    Eigen::Quaterniond w_quat, quat_t_inverse;
    quat_t_inverse = quat_t.inverse();

    w_quat = dquat_t * quat_t_inverse;
    w_quat.w() *= 2;
    w_quat.vec() *= 2;

    w_t = w_quat.vec();

    dw_t = (ddquat_t * quat_t_inverse).vec()*2 - (w_quat * dquat_t * quat_t_inverse).vec();

}

void quintic_traj::computeCoeffs_Linear (){ //TBD: simplify
  // Major part of this function is generated automatically by sympy. Manual edition shall be avoided.


//     clock_t tic = clock();

    t0_2 = pow(t0, 2);
    t0_3 = pow(t0, 3);
    t0_4 = pow(t0, 4);
    t0_5 = pow(t0, 5);
    tf_2 = pow(tf, 2);
    tf_3 = pow(tf, 3);
    tf_4 = pow(tf, 4);
    tf_5 = pow(tf, 5);

//     A_inv.resize(6, 6);
    A_inv(0, 0) = tf_3*(-10*t0_2 + 5*t0*tf - tf_2)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(0, 1) = t0*tf_3*(4*t0 - tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(0, 2) = -t0_2*tf_3/(2*t0_3 - 6*t0_2*tf + 6*t0*tf_2 - 2*tf_3);
    A_inv(0, 3) = t0_3*(t0_2 - 5*t0*tf + 10*tf_2)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(0, 4) = -t0_3*tf*(t0 - 4*tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(0, 5) = (1.0L/2.0L)*t0_3*tf_2/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(1, 0) = 30*t0_2*tf_2/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(1, 1) = tf_2*(-12*t0_2 - 4*t0*tf + tf_2)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(1, 2) = (1.0L/2.0L)*t0*tf_2*(3*t0 + 2*tf)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(1, 3) = -30*t0_2*tf_2/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(1, 4) = t0_2*(t0_2 - 4*t0*tf - 12*tf_2)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(1, 5) = -t0_2*tf*(2*t0 + 3*tf)/(2*t0_3 - 6*t0_2*tf + 6*t0*tf_2 - 2*tf_3);
    A_inv(2, 0) = -30*t0*tf*(t0 + tf)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(2, 1) = 6*t0*tf*(2*t0 + 3*tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(2, 2) = -tf*(3*t0_2 + 6*t0*tf + tf_2)/(2*t0_3 - 6*t0_2*tf + 6*t0*tf_2 - 2*tf_3);
    A_inv(2, 3) = 30*t0*tf*(t0 + tf)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(2, 4) = 6*t0*tf*(3*t0 + 2*tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(2, 5) = (1.0L/2.0L)*t0*(t0_2 + 6*t0*tf + 3*tf_2)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(3, 0) = 10*(t0_2 + 4*t0*tf + tf_2)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(3, 1) = -(4*t0_2 + 20*t0*tf + 6*tf_2)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(3, 2) = (1.0L/2.0L)*(t0_2 + 6*t0*tf + 3*tf_2)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(3, 3) = -(10*t0_2 + 40*t0*tf + 10*tf_2)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(3, 4) = -(6*t0_2 + 20*t0*tf + 4*tf_2)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(3, 5) = -(3*t0_2 + 6*t0*tf + tf_2)/(2*t0_3 - 6*t0_2*tf + 6*t0*tf_2 - 2*tf_3);
    A_inv(4, 0) = -(15*t0 + 15*tf)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(4, 1) = (7*t0 + 8*tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(4, 2) = -(t0 + (3.0L/2.0L)*tf)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(4, 3) = 15*(t0 + tf)/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(4, 4) = (8*t0 + 7*tf)/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(4, 5) = ((3.0L/2.0L)*t0 + tf)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(5, 0) = 6/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(5, 1) = -3/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(5, 2) = (1.0L/2.0L)/(t0_3 - 3*t0_2*tf + 3*t0*tf_2 - tf_3);
    A_inv(5, 3) = -6/(t0_5 - 5*t0_4*tf + 10*t0_3*tf_2 - 10*t0_2*tf_3 + 5*t0*tf_4 - tf_5);
    A_inv(5, 4) = -3/(t0_4 - 4*t0_3*tf + 6*t0_2*tf_2 - 4*t0*tf_3 + tf_4);
    A_inv(5, 5) = -1/(2*t0_3 - 6*t0_2*tf + 6*t0*tf_2 - 2*tf_3);

//     b.resize(6);
//     p_Linear.resize(3, 6);

    for (int i = 0; i < 3; i++){
    b << x0[i], dx0[i], ddx0[i], xf[i], dxf[i], ddxf[i];
//     std::cout << x0[i]<< dx0[i]<< ddx0[i]<< xf[i]<< dxf[i]<< ddxf[i] << std::endl;
    _p_Linear_i.noalias() = A_inv * b;
    p_Linear.row(i) = _p_Linear_i;
    }

//     clock_t toc = clock();
//     std::cout << "Elapsed time for generation of trajectory: " << double(toc - tic) / CLOCKS_PER_SEC << std::endl;

return;
}


void quintic_traj::computeCoeffs_Angular (bool first_step){

    double T = tf - t0_updated;

    if (traj_type_Angular == "quintic"){

      if (!first_step){
      	double ddN;
      	quintic_traj::computeDDN(2*dt/(T+dt), T+dt, ddN);
      }

      p10_eigen = Eigen::Quaterniond(0, w_old[0]/2, w_old[1]/2, w_old[2]/2);
      p20_eigen = Eigen::Quaterniond(-.25*pow(w_old.norm(), 2) + ddN, dw_old[0]/2, dw_old[1]/2, dw_old[2]/2);

      Eigen::Quaterniond dquat0 = p10_eigen * quat_old;  // equ(6)
      Eigen::Quaterniond ddquat0 = p20_eigen * quat_old; // equ(7)

      //// equ(13):
      p_Angular0 = quat_old;
      p_Angular1.coeffs().noalias() = 3*quat_old.coeffs() + dquat0.coeffs()*T;
      p_Angular2.coeffs().noalias() = (ddquat0.coeffs()*T*T + 6*dquat0.coeffs()*T + 12*quat_old.coeffs())/2;
      p_Angular3 = quatf;
      p_Angular4.coeffs().noalias() = 3*quatf.coeffs() - dquatf.coeffs()*T;
      p_Angular5.coeffs().noalias() = (ddquatf.coeffs()*T*T - 6*dquatf.coeffs()*T + 12*quatf.coeffs())/2;

      p_Angular[0] = p_Angular0;
      p_Angular[1] = p_Angular1;
      p_Angular[2] = p_Angular2;
      p_Angular[3] = p_Angular3;
      p_Angular[4] = p_Angular4;
      p_Angular[5] = p_Angular5;

    }
    else { //traj_type_Angular == "cubic"
      p10_eigen = Eigen::Quaterniond(0, w_old[0]/2, w_old[1]/2, w_old[2]/2);
      p20_eigen = Eigen::Quaterniond(-.25*pow(w_old.norm(), 2), dw_old[0]/2, dw_old[1]/2, dw_old[2]/2);

      Eigen::Quaterniond dquat0 = p10_eigen * quat_old;
      Eigen::Quaterniond ddquat0 = p20_eigen * quat_old;

      p_Angular0 = quat_old;
      p_Angular1 = quatf;

      mu.w() = p_Angular1.w() - p_Angular0.w();
      mu.vec() = p_Angular1.vec() - p_Angular0.vec();
      mu = mu.inverse();
      mu0 = mu*dquat0;
      muf = mu*dquatf;
      mu0.vec() *= T;
      mu0.w() *= T;
      muf.vec() *= T;
      muf.w() *= T;

      p_Angular2.w() = mu0.w() + muf.w() - 2;
      p_Angular2.vec() = mu0.vec() + muf.vec();

      p_Angular3.w() = -2*mu0.w() - muf.w() + 3;
      p_Angular3.vec() = -2*mu0.vec() - muf.vec();

      p_Angular4 = mu0;

      p_Angular.clear();
      p_Angular.push_back(p_Angular0);
      p_Angular.push_back(p_Angular1);
      p_Angular.push_back(p_Angular2);
      p_Angular.push_back(p_Angular3);
      p_Angular.push_back(p_Angular4);

    }

    p_Angular_old = p_Angular;

return;
}


void quintic_traj::Quaterniond2Vector4d(const Eigen::Quaterniond& quat, Eigen::Vector4d& vector){
  vector[0] = quat.w();
  vector[1] = quat.x();
  vector[2] = quat.y();
  vector[3] = quat.z();
}

void quintic_traj::transformPhase2Time(double tau, double& tau01, double& dtau01, double& ddtau01){
  tau01 = pow(tau, 3)*(6*tau*tau - 15*tau + 10);
  dtau01 = 30*tau*tau*(tau*tau - 2*tau + 1);
  ddtau01 = 60*tau*(2*tau*tau - 3*tau + 1);
}

void quintic_traj::computeSlerp(double t,
		  Eigen::Quaterniond& s, Eigen::Quaterniond& ds, Eigen::Quaterniond& dds){
//   s = quat0.slerp(t, quatf);
  quintic_traj::computeSlerpSelf(t, quat0, quatf, s);
//   std::cout << Logq0invq1.coeffs() << std::endl;
  ds = s*Logq0invq1;
  dds = ds*Logq0invq1;
}

void quintic_traj::computeQuaternionLog(const Eigen::Quaterniond& q, Eigen::Quaterniond& logq){

  logq.w() = 0;
  double angle = 2*std::acos(q.w());
  double sin_angle = std::sin(angle/2);
  if (fabs(sin_angle) > 1e-7)
    logq.vec() = angle/2*q.vec()/sin_angle;
  else
    logq.vec() = q.vec();
}

// void quintic_traj::computeQuaternionLog(const Eigen::Quaterniond& q, Eigen::Quaterniond& logq){
//    double v_norm = q.vec().norm();
//    double q_norm = q.coeffs().norm();
//    double tolerance = 1e-17;
//
//    if (q_norm < tolerance){
//       logq.w() = q.w()*0;
//       logq.vec() = q.vec()*0;
//       return;
//    }
//
//    if (v_norm < tolerance){
//         // real quaternions - no imaginary part
//       logq.w() = std::log(q_norm);
//       logq.vec().setZero();
//       return;
//    }
//
//     logq.w() = q_norm;
//     logq.vec() = std::acos(q.w()/q_norm)*q.vec() / v_norm;
//     return;
//
// }

void quintic_traj::computeQuaternionExp(const Eigen::Quaterniond& q, Eigen::Quaterniond& expq){

  double angle = std::sqrt(q.vec().dot(q.vec()));
  double sin_angle = std::sin(angle);
  expq.w() = std::cos(angle);
  if (fabs(sin_angle) > 1e-7)
    expq.vec().noalias() = q.vec()*sin_angle/angle;
  else
    expq.vec().noalias() = q.vec();
}

// void quintic_traj::computeQuaternionExp(const Eigen::Quaterniond& q, Eigen::Quaterniond& expq){
//
//   double tolerance = 1e-17;
//   double v_norm = q.vec().norm();
//   Eigen::Vector3d vec = q.vec();
//     if (v_norm > tolerance)
//       vec = vec / v_norm;
//
//     expq.w() = std::exp(q.w())*std::cos(v_norm);
//     expq.vec() = std::exp(q.w())*std::sin(v_norm)*vec;
// }


void quintic_traj::computeSquad(double t,
		  Eigen::Quaterniond& s, Eigen::Quaterniond& ds, Eigen::Quaterniond& dds){

  quintic_traj::computeSquadAux(t, quat0, quatf, squad_a, squad_b, s);

  ds.w() = s.w()*0;
  ds.vec() = s.vec()*0;
  dds.w() = ds.w()*0;
  dds.vec() = ds.vec()*0;
}

void quintic_traj::computeSquadAux(double t, const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
					     const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
					     Eigen::Quaterniond& squad)
{

//   std::cout << "b: " << b.coeffs().transpose() << std::endl;


//   r1 = q.slerp(t, p);
  quintic_traj::computeSlerpSelf(t, q, p, r1);
//   r2 = a.slerp(t, b);
  quintic_traj::computeSlerpSelf(t, a, b, r2);

//   squad = (q.slerp(t, p)).slerp(2*t*(1 - t), a.slerp(t, b));
  quintic_traj::computeSlerpSelf(2*t*(1 - t), r1, r2, squad);
//   std::cout << b.coeffs().transpose() << std::endl;
//   squad = a.slerp(t, b);
}

void quintic_traj::computeSquadICs(const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
                                   const Eigen::Vector3d omega_0, const Eigen::Vector3d omega_f,
				   const double T,
				   Eigen::Quaterniond& squad_a, Eigen::Quaterniond& squad_b)
{
  omega_0_quat.w() = 0;
  omega_0_quat.vec().noalias() = omega_0;
  omega_f_quat.w() = 0;
  omega_f_quat.vec().noalias() = omega_f;

  q_temp1.coeffs().noalias() = (q.inverse()*omega_0_quat*q).coeffs()*T/2;
//   q_temp1.vec() = (q.inverse()*omega_0_quat*q).vec()*T/2;

  q_temp2.coeffs().noalias() = (p.inverse()*omega_f_quat*p).coeffs()*T/2;
//   q_temp2.vec() = (p.inverse()*omega_f_quat*p).vec()*T/2;

  quintic_traj::computeQuaternionLog(q.inverse()*p, term_log);
  term_exp1.coeffs().noalias() = (q_temp1.coeffs() - term_log.coeffs())/2;
//   term_exp1.vec() = (q_temp1.vec() - term_log.vec())/2;
  term_exp2.coeffs().noalias() = (-q_temp2.coeffs() + term_log.coeffs())/2;
//   term_exp2.vec() = (-q_temp2.vec() + term_log.vec())/2;

  quintic_traj::computeQuaternionExp(term_exp1, exp1);
  quintic_traj::computeQuaternionExp(term_exp2, exp2);

  squad_a = q*exp1;
  squad_b = p*exp2;
}

void quintic_traj::computeQuaternionPower(const Eigen::Quaterniond& q, const double t, Eigen::Quaterniond& q_power_t){

  quintic_traj::computeQuaternionLog(q, logq);
  logq.coeffs() *= t;
  quintic_traj::computeQuaternionExp(logq, q_power_t);
}


void quintic_traj::computeSlerpSelf(const double t, const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
				    Eigen::Quaterniond& sl)
{
  quintic_traj::computeQuaternionPower(q.inverse() * p, t, expq);
  sl = q * expq;

}
/*
  def __pow__(self, exponent):
        # source: https://en.wikipedia.org/wiki/Quaternion#Exponential.2C_logarithm.2C_and_power
        exponent = float(exponent) # Explicitly reject non-real exponents
        norm = self.norm
        if norm > 0.0:
            try:
                n, theta = self.polar_decomposition
            except ZeroDivisionError:
                # quaternion is a real number (no vector or imaginary part)
                return Quaternion(scalar=self.scalar ** exponent)
            return (self.norm ** exponent) * Quaternion(scalar=cos(exponent * theta), vector=(n * sin(exponent * theta)))
        return Quaternion(self)

}*/







/*
void quintic_traj::computeQuaternionLog(const Eigen::Quaterniond& q, Eigen::Quaterniond& logq){
 double angle = 2*std::acos(q.w());
 logq.vec() = angle/2*q.vec()/std::sin(angle/2);
}*/



/*
void transformDPhase2Time(double tau, double& tau01){
  tau01 = 30*tau*tau*(tau*tau - 2*tau + 1);
}


void transformDDPhase2Time(double tau, double& tau01){
  tau01 = 60*tau*(2*tau*tau - 3*tau + 1);
}  */


// def computePhas2Phase(t): return t**3*(6*t**2 - 15*t + 10)
// def computeDPhas2Phase(t): return 30*t**2*(t**2 - 2*t + 1)
// def computeDDPhas2Phase(t): return 60*t*(2*t**2 - 3*t + 1)
//
// def computeSlerp(t, q0, q1):
//     th = np.arccos(q0.q.dot(q1.q))
//     c0 = np.sin(th*(1 - t))/np.sin(th)
//     c1 = np.sin(t*th)/np.sin(th)
//     return c0*q0 + c1*q1
//
// def computeDSlerp(t, q0, q1):
//     return computeSlerp(t, q0, q1)*quat_log(q0.inverse*q1)
//
// def computeDDSlerp(t, q0, q1):
//     return computeSlerp(t, q0, q1)*quat_log(q0.inverse*q1)*quat_log(q0.inverse*q1)
//
// def quat_log(q):
//     if q.is_unit():
//         angle = q.angle
//         return Quaternion(vector=angle/2*q.vector/np.sin(angle/2))





void quintic_traj::solveLinearSys_abcd(const Eigen::VectorXd& a,
			 const Eigen::VectorXd& b,
                         const Eigen::VectorXd& c,
                         const Eigen::VectorXd& d,
                               Eigen::VectorXd& f){

  size_t N = d.size();

//   Eigen::VectorXd cp(N-1), dp(N);

  cp[0] = c[0] / b[0];
  dp[0] = d[0] / b[0];

  for (int i=1; i<N; i++) {
    double m = 1.0 / (b[i] - a[i-1] * cp[i-1]);
    if (i < N - 1) cp[i] = c[i] * m;
    dp[i] = (d[i] - a[i-1] * dp[i-1]) * m;
  }

  f[N-1] =  dp[N-1];

  for (int i=N-1; i-- > 0; ) {
    f[i] = dp[i] - cp[i] * f[i+1];
  }
}

void quintic_traj::computeSquadSplineControlPoints(){

  t_all = t_viapts;
  t_all.push_back(tf);
  t_all.insert(t_all.begin(), t0);

  quat_all = quat_viapts;
  quat_all.push_back(quatf);
  quat_all.insert(quat_all.begin(), quat0);

  Eigen::Vector3d w_dummy(0., 0., 0.);
  Eigen::Quaterniond quat_1, quat_2, quat_3;


  size_t N = t_all.size();

  //first point:
  double T = t_all[1] - t_all[0];
  quintic_traj::computeSquadICs(quat_all[0], quat_all[1], w0, w_dummy, T, quat_1, quat_2);
  quat_cpoints.push_back(quat_1);

  //via points:
  for (int j=1; j < N - 1; j++)
  {
    quintic_traj::computeQuaternionLog(quat_all[j].inverse() * quat_all[j+1], quat_1);
    quintic_traj::computeQuaternionLog(quat_all[j].inverse() * quat_all[j-1], quat_2);
    quat_3.coeffs() = -.25*(quat_1.coeffs() + quat_2.coeffs());
    quintic_traj::computeQuaternionExp(quat_3, quat_1);
    quat_cpoints.push_back(quat_all[j] * quat_1);
  }

  //last point:
  T = t_all[N - 1] - t_all[N - 2];
  quintic_traj::computeSquadICs(quat_all[N - 2], quat_all[N - 1], w_dummy, wf, T, quat_1, quat_2);
  quat_cpoints.push_back(quat_2);

//   std::cout << quat_cpoints[0].coeffs().transpose() << std::endl;
//   std::cout << quat_cpoints[1].coeffs().transpose() << std::endl;
//   std::cout << quat_cpoints[2].coeffs().transpose() << std::endl;

}


void quintic_traj::getSquadSplineCurrentParams(const double t,
				 std::vector<Eigen::Quaterniond>& quats,
				 double& t0_here, double& T_here)
{
  int  ind = 0;
  size_t N = t_all.size();
  if (t > t_all[1] && t <= t_all[N - 2]){
    for (int j= 1; j < N - 2; j++){
      if (t <= t_all[j+1] && t > t_all[j]){
	ind = j;
	break;
      }
    }

  }else if(t > t_all[N - 2]) ind = N - 2;


//   quats.clear();
  quats[0] = quat_all[ind];
  quats[1] = quat_all[ind + 1];
  quats[2] = quat_cpoints[ind];
  quats[3] = quat_cpoints[ind + 1];

  t0_here = t_all[ind];
  T_here = t_all[ind+1] - t_all[ind];

//   std::cout << quat_all[0].coeffs() << std::endl;
//   std::cout << quat_all[1].coeffs() << std::endl;
//   std::cout << quat_all[2].coeffs() << std::endl;
}

void quintic_traj::computeDSquad_num(const double& t,
				     const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
				     const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
                                     Eigen::Quaterniond& ds)
{
  Eigen::Quaterniond s1, s2, s3;

  if(fabs(t) < 1e-6){
//     std::cout << "we arrived here at time + " << t << std::endl;
    quintic_traj::computeQuaternionLog(q.inverse() * p, s1);
    quintic_traj::computeQuaternionLog(q.inverse() * a, s2);
    s3.coeffs() = s1.coeffs() + 2*s2.coeffs();
    ds = q * s3;
    return;
  }
  else if(fabs(1 - t) < 1e-6){
//     std::cout << "we arrived here at time * " << t << std::endl;
    quintic_traj::computeQuaternionLog(q.inverse() * p, s1);
    quintic_traj::computeQuaternionLog(p.inverse() * b, s2);
    s3.coeffs() = s1.coeffs() - 2*s2.coeffs();
    ds = p * s3;
    return;
  }
//   std::cout << "we arrived here at time  " << t << std::endl;
  double dt = 1e-3;
  quintic_traj::computeSquadAux(t - dt/2, q, p, a, b, s1);
  quintic_traj::computeSquadAux(t + dt/2, q, p, a, b, s2);
  ds.coeffs() = (s2.coeffs() - s1.coeffs())/dt;

}

// if t == 0: return q0*(quat_log(q0.inverse*q1) + 2*quat_log(q0.inverse*a))
//     elif t == 1: return q1*(quat_log(q0.inverse*q1) - 2*quat_log(q1.inverse*b))
//
//     sq1 = computeSquadAux(t - dt/2, q0, q1, a, b)
//     sq2 = computeSquadAux(t + dt/2, q0, q1, a, b)
//     return (sq2 - sq1)/dt








void quintic_traj::getPolySplineCurrentParams(const double& t,
					      const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d dw,
				              QCoeffs& p, double& t0_here)
{

  ind_poly = 0;

  if ((t + dt) >= t_all[1] && (t + dt) < t_all[N - 2]){
    for (int j= 1; j < N - 2; j++){
      if ((t + dt) < t_all[j+1] && (t + dt) >= t_all[j]){
      	ind_poly = j;
      	break;
      }
    }
  }
  else if((t + dt) >= t_all[N - 2]) ind_poly = N - 2;

  p = VQcoeffs[ind_poly];
  t0_here = t_all[ind_poly];

  if (traj_Angular_successive)
    quintic_traj::ifSRPoly(t, q, w, dw, p, t0_here);

}


void quintic_traj::ifSRPoly(const double& t, const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d dw,
			    QCoeffs& p, double& t0_here)
{
    t_all_updated = t_all;
    quat_all_updated = quat_all;
    t_all_updated[ind_poly] = t;
    quat_all_updated[ind_poly] = q;


    double dN, ddN;
    quintic_traj::computeQuatNormDerivs(qcoeffs_old, 2*dt, dN, ddN);  // equ(17, 18)
    dN = 0;

    //// equ(21):
    dq_sr.vec().noalias() = w/2;
    dq_sr.w() = dN/2;
    dq_sr = dq_sr * q;

    //// equ(22):
    ddq_sr.vec().noalias() = dw/2 + w*dN;
    ddq_sr.w() = -.25*pow(w.norm(), 2) + ddN;
    ddq_sr = ddq_sr * q;

    dqddq.dq0 = dq_sr;
    dqddq.ddq0 = ddq_sr;

    std::string stype;
    if (ind_poly == 0) {stype = "434";}
    else if (ind_poly == N - 2) {stype = "004";}
    else {stype = "334";}

    quintic_traj::computePolySplineCpoints(t_all_updated, quat_all_updated, stype);



    p = VQcoeffs[ind_poly];
    t0_here = t;
  }

void quintic_traj::computeCoeffs_first(const double& h, const double& dvi,
                                       Eigen::Vector2d& a3a4)
{
  m22 << 4./pow(h, 3), -1./(h*h), -3./pow(h, 4), 1./pow(h, 3);
  vec2 << pv.xii - pv.xi - pv.vi*h - dvi/2.*h*h, pv.vii - pv.vi - dvi*h;

  a3a4.noalias() = m22 * vec2;
}

void quintic_traj::computeCoeffs_last(const double& h, const double& dvii,
                                       Eigen::Vector3d& a2a3a4)
{
  m33 << 6./(h*h), -3./h, 1./2, -8./pow(h, 3), 5./(h*h), -1./h, 3./pow(h, 4), -2./pow(h, 3), 1./(2*h*h);
  vec3 << pv.xii - pv.xi - pv.vi*h, pv.vii - pv.vi, dvii;

  a2a3a4.noalias() = m33 * vec3;
}

void quintic_traj::computeCoeffs_general(const double& h, Eigen::Vector2d& a2a3)
{
  m22 << 3./(h*h), -1./h, -2./pow(h, 3), 1./(h*h);
  vec2 << pv.xii - pv.xi - pv.vi*h, pv.vii - pv.vi;

  a2a3.noalias() = m22 * vec2;
}

void quintic_traj::computePolySplineCPointsAux(const std::vector<double>& t, const Eigen::VectorXd& x,
				               const bool& eval_A, const std::string& stype)
{

  if (stype == "004")
    {
      pv = (PosVel){x[N - 2], x[N - 1], vdv.v0, vdv.vN};
      quintic_traj::computeCoeffs_last(t[N - 1] - t[N - 2], vdv.dvN, vec3_temp);
      Vcoeffs[N - 2] = (Coeffs){x[N - 2], vdv.v0, vec3_temp[0], vec3_temp[1], vec3_temp[2]};
      return;
    }

  quintic_traj::computePolySplineV(t, x, eval_A, stype);  //overrides v_all

  for (int i = ind_poly; i < N - 1; i++)
    {
    	if (i == ind_poly)
    	  {
      	  if (stype == "334")
      	    {
              // std::cout << "general 1" << std::endl;
      	      pv = (PosVel){x[i], x[i+1], v_all[i], v_all[i+1]};
      	      quintic_traj::computeCoeffs_general(t[i+1] - t[i], vec2_temp);
      	      Vcoeffs[i] = (Coeffs){x[i], v_all[i], vec2_temp[0], vec2_temp[1], NAN};
      	    }
      	  else
      	    {
              // std::cout << "first" << std::endl;
      	      pv = (PosVel){x[i], x[i+1], vdv.v0, v_all[i+1]};
      	      quintic_traj::computeCoeffs_first(t[i+1] - t[i], vdv.dv0, vec2_temp);
      	      Vcoeffs[i] = (Coeffs){x[i], v_all[i], vdv.dv0/2., vec2_temp[0], vec2_temp[1]};
      	    }
    	  }
    	else if (i == N - 2)
    	  {
          // std::cout << "last" << std::endl;
    	    pv = (PosVel){x[i], x[i+1], v_all[i], v_all[i+1]};
    	    quintic_traj::computeCoeffs_last(t[i+1] - t[i], vdv.dvN, vec3_temp);
    	    Vcoeffs[i] = (Coeffs){x[i], v_all[i], vec3_temp[0], vec3_temp[1], vec3_temp[2]};
    	  }
    	else
    	  {
          // std::cout << "general 2" << std::endl;
    	    pv = (PosVel){x[i], x[i+1], v_all[i], v_all[i+1]};
    	    quintic_traj::computeCoeffs_general(t[i+1] - t[i], vec2_temp);
    	    Vcoeffs[i] = (Coeffs){x[i], v_all[i], vec2_temp[0], vec2_temp[1], NAN};
    	  }

    }
}


void quintic_traj::computePolySplineV(const std::vector<double>& t, const Eigen::VectorXd& x,
				      const bool& eval_A, const std::string& stype)
{

//   size_t N = t.size();

  N_updated = N - ind_poly;

//   Eigen::VectorXd vv(N_updated - 2);

  if (N_updated == 3)  // when only 1 via point
    {
      double h1 = t[N - 2] - t[ind_poly], h2 = t[N - 1] - t[N - 2];
      if (stype == "334") v_temp[0] = 1/(h1*h2*(3*h1 + 2*h2))*(6*h1*h1*(x[N - 1] - x[N - 2]) + 3*h2*h2*(x[N - 2] - x[ind_poly])
	                        - h1*h2*h2*vdv.v0 - 3*h1*h1*h2*vdv.vN + 1./2*h1*h1*h2*h2*vdv.dvN); // srpoly
      else v_temp[0] = 1./(h1*h2*(h1 + h2))*(2*h1*h1*(x[2] - x[1]) + 2*h2*h2*(x[1] - x[0]) - h1*h2*h2*vdv.v0
	         - h1*h1*h2*vdv.vN + 1./6*h1*h1*h2*h2*(vdv.dvN - vdv.dv0));  //npoly

      v_all.tail(3) << vdv.v0, v_temp[0], vdv.vN;
    }
  else // when more than 1 via points
    {
      if (eval_A)
      {
	abcd.a.setZero(); //TODO: resize vectors in abcd in initialization
	abcd.b.setZero();
	abcd.c.setZero();
      }
      abcd.d.setZero();

//       int count = 0;

      for (int i=ind_poly; i < N - 2; i++)
      {
// 	  count++;
	  double hi = t[i+1] - t[i], hii= t[i+2] - t[i+1];
	  if (i == ind_poly)
	  {
	    if (eval_A)
	    {
	      abcd.a[i] = NAN; //fake
	      if (stype == "334") abcd.b[i] = 2*(hi + hii);
	      else abcd.b[i] = 2*hi + 3*hii;
	      abcd.c[i] = hi;
	    }
	    if (stype == "334") abcd.d[i] = 3/(hi*hii)*(hi*hi*(x[i+2] - x[i+1]) + hii*hii*(x[i+1] - x[i])) - hii*vdv.v0;
	    else abcd.d[i] = 3/(hi*hii)*(hi*hi*(x[i+2] - x[i+1]) + 2*hii*hii*(x[i+1] - x[i])) - 3*hii*vdv.v0 - 1./2*hi*hii*vdv.dv0;
	  }
	  else if (i == N - 3)
	  {
	    if (eval_A)
	    {
	      abcd.a[i] = hii;
	      abcd.b[i] = 3*hi + 2*hii;
	      abcd.c[i] = NAN; //fake
	    }
	    abcd.d[i] = 3/(hi*hii)*(2*hi*hi*(x[i+2] - x[i+1]) + hii*hii*(x[i+1] - x[i])) - 3*hi*vdv.vN + 1./2*hi*hii*vdv.dvN;
	  }
	  else
	  {
	    if (eval_A)
	    {
	      abcd.a[i] = hii;
	      abcd.b[i] = 2*(hi + hii);
	      abcd.c[i] = hi;
	    }
	  abcd.d[i] = 3/(hi*hii)*(hi*hi*(x[i+2] - x[i+1]) + hii*hii*(x[i+1] - x[i]));
	  }

      }

//     if (eval_A) //TODO: omitted this vs python code
//     {
// //       abcd.a_resized = abcd.a.tail(N - 3);
// //       abcd.c_resized = abcd.c.head(N - 3);
//     }
//   vv.resize(N_updated - 2);
//   Eigen::VectorXd vv = abcd.b.tail(count);
//   std::cout<< "count " << count << std::endl;

  quintic_traj::solveLinearSys_abcd(abcd.a.tail(N_updated - 3), abcd.b.tail(N_updated - 2),
				    (abcd.c.tail(N_updated - 2)).head(N_updated - 3), abcd.d.tail(N_updated - 2), v_temp);

  //   v_all << vdv.v0, vv, vdv.vN;
  v_all.tail(N_updated) << vdv.v0, v_temp.head(N_updated - 2), vdv.vN;
  }
}



void quintic_traj::computePolySplineCpoints(const std::vector<double>& t, const std::vector<Eigen::Quaterniond>& x,
					    std::string stype)
{

  bool eval_A = true;
  for (int j = 0; j<4; j++)
  {

    for (int i = 0; i < N ; i++) quat_elements_temp[i] = x[i].coeffs()[j];

    vdv = (VdV){dqddq.dq0.coeffs()[j], dqddq.dqf.coeffs()[j], dqddq.ddq0.coeffs()[j], dqddq.ddqf.coeffs()[j]};
    quintic_traj::computePolySplineCPointsAux(t, quat_elements_temp, eval_A, stype);

    for (int i = ind_poly; i < N - 1 ; i++) //TODO: VQcoeffs should be passed by the correct size (N - 1)
    {
    	VQcoeffs[i].p0.coeffs()[j] = Vcoeffs[i].a0;
    	VQcoeffs[i].p1.coeffs()[j] = Vcoeffs[i].a1;
    	VQcoeffs[i].p2.coeffs()[j] = Vcoeffs[i].a2;
    	VQcoeffs[i].p3.coeffs()[j] = Vcoeffs[i].a3;
    	VQcoeffs[i].p4.coeffs()[j] = Vcoeffs[i].a4;
    }

    if (j == 0) eval_A = false;
  }
}

void quintic_traj::computeCubicTraj(const QCoeffs& p, const double& tau,
                                    Eigen::Quaterniond& q, Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq)
{
  q.coeffs().noalias() = p.p0.coeffs() + p.p1.coeffs()*tau + p.p2.coeffs()*tau*tau + p.p3.coeffs()*pow(tau, 3);
  dq.coeffs().noalias() = p.p1.coeffs() + 2*p.p2.coeffs()*tau + 3*p.p3.coeffs()*pow(tau, 2);
  ddq.coeffs().noalias() = 2*p.p2.coeffs() + 6*p.p3.coeffs()*tau;
}

void quintic_traj::computeQuarticTraj(const QCoeffs& p, const double& tau,
                                    Eigen::Quaterniond& q, Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq)
{
  q.coeffs().noalias() = p.p0.coeffs() + p.p1.coeffs()*tau + p.p2.coeffs()*tau*tau +
               p.p3.coeffs()*pow(tau, 3) + p.p4.coeffs()*pow(tau, 4);
  dq.coeffs().noalias() = p.p1.coeffs() + 2*p.p2.coeffs()*tau + 3*p.p3.coeffs()*pow(tau, 2) +
                4*p.p4.coeffs()*pow(tau, 3);
  ddq.coeffs().noalias() = 2*p.p2.coeffs() + 6*p.p3.coeffs()*tau + 12*p.p4.coeffs()*pow(tau, 2);
}


void quintic_traj::computeQuatNormDerivs(const QCoeffs& p, const double& t,
					 double& dN, double& ddN)
{

  if (std::isnan(p.p4.w())){
     // std::cout << "cubic" << std::endl;  // second jump here.
     quintic_traj::computeCubicTraj(p, t, q_norm, dq_norm, ddq_norm);
   }
   else{
     // std::cout << "quintic" << std::endl;  // first jump here.
     quintic_traj::computeQuarticTraj(p, t, q_norm, dq_norm, ddq_norm);
   }
  double n = q_norm.norm();
  dN = 1/n*q_norm.coeffs().dot(dq_norm.coeffs());  // equ(17)
  ddN =1/n * (q_norm.coeffs().dot(ddq_norm.coeffs()) + dq_norm.coeffs().dot(dq_norm.coeffs()) - dN*dN);  // equ(18)

}



} //namespace traj_gen



//       double h = t[1] - t[0];
//       coeffs.a0 = x[0];
//       coeffs.a1 = v0;
//       posvel_temp.xi = x[0];
//       posvel_temp.xii = x[1];
//       posvel_temp.vi = v0;
//       posvel_temp.vii = vN;
//       posvel_temp = (PosVel){x[0], x[1], v0, vN};

//       coeffs.a2 = vec3_temp[0];
//       coeffs.a3 = vec3_temp[1];
//       coeffs.a4 = vec3_temp[2];
//       coeffs = (Coeffs){};
