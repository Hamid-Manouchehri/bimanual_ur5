/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Mohammad Shahbazi
 * email:  mohammad.shahbazi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __TRAJ6D_H__
#define __TRAJ6D_H__

#include "eigen3/Eigen/Dense"
#include <vector>
// #include "Quaternions/Quaternions.hpp"



namespace traj_gen{

class quintic_traj{
public:

  quintic_traj();

  void setInitialPosition_Linear(const Eigen::Vector3d& init_pose);
  void setInitialVelocity_Linear(const Eigen::Vector3d& init_vel);
  void setInitialAcceleration_Linear(const Eigen::Vector3d& init_acc);

  void setInitialPosition_Angular(const Eigen::Quaterniond& init_quat);
  void setInitialVelocity_Angular(const Eigen::Vector3d& init_angular_vel);
  void setInitialAcceleration_Angular(const Eigen::Vector3d& init_angular_acc);

  void setInitialTime(const double& init_time);

  void setFinalPosition_Linear(const Eigen::Vector3d& final_pos);
  void setFinalVelocity_Linear(const Eigen::Vector3d& final_vel);
  void setFinalAcceleration_Linear(const Eigen::Vector3d& final_acc);

  void setFinalPosition_Angular(const Eigen::Quaterniond& final_quat);
  void setFinalVelocity_Angular(const Eigen::Vector3d& final_angular_vel);
  void setFinalAcceleration_Angular(const Eigen::Vector3d& final_angular_acc);

  void setFinalTime(const double& final_time);

  void generateTraj_Linear(const std::string& type);
  void generateTraj_Angular(const std::string& type, const bool method_successive = false, const double time_step = 0.001);

  void getState_Linear(const double t, Eigen::Vector3d& x, Eigen::Vector3d& dx, Eigen::Vector3d& ddx);
  void getState_Angular(const double t, Eigen::Quaterniond& quat, Eigen::Vector3d& w,
						    Eigen::Vector3d& dw, const double period = 0.001);

  void setViaPointsTime(const std::vector<double>& viapoints_time);
  void setViaPointsPosition_Angular(const std::vector<Eigen::Quaterniond>& viapoints_quat);

//   void getPosition_Linear(const double t, Eigen::Vector3d& x, bool update=false);
//   void getVelocity_Linear(const double t, Eigen::Vector3d& dx, bool update=false);
//   void getAcceleration_Linear(const double t, Eigen::Vector3d& ddx, bool update=false);
//
//   void getPosition_Angular(const double t, Eigen::Quaterniond& quat, bool update=false);
//   void getVelocity_Angular(const double t, Eigen::Vector3d& w, bool quat_update = true, bool update=false);
//   void getAcceleration_Angular(const double t, Eigen::Vector3d& dw, bool quat_update =true,
// 					   bool dquat_update=true, bool update=false);

  void computeReferenceAcceleration(const double t, const Eigen::Affine3d& actual_pose,
				    const Eigen::VectorXd& actual_vel,
				    Eigen::VectorXd& reference_acc);

  void computeReferenceError(const double t, const Eigen::Affine3d& actual_pose,
				    const Eigen::VectorXd& actual_vel,
				    Eigen::VectorXd& reference_err,
				    Eigen::VectorXd& reference_derr
			    );

  void getInitialPosition_Linear(Eigen::Vector3d& x);
  void getInitialVelocity_Linear(Eigen::Vector3d& dx);
  void getInitialAcceleration_Linear(Eigen::Vector3d& ddx);

  void getInitialPosition_Angular(Eigen::Quaterniond& quat);
  void getInitialVelocity_Angular(Eigen::Vector3d& w);
  void getInitialAcceleration_Angular(Eigen::Vector3d& dw);

  void getFinalPosition_Linear(Eigen::Vector3d& x);
  void getFinalVelocity_Linear(Eigen::Vector3d& dx);
  void getFinalAcceleration_Linear(Eigen::Vector3d& ddx);

  void getFinalPosition_Angular(Eigen::Quaterniond& quat);
  void getFinalVelocity_Angular(Eigen::Vector3d& w);
  void getFinalAcceleration_Angular(Eigen::Vector3d& dw);

  void Quaternion2Euler(const Eigen::Quaterniond& quat, Eigen::Vector3d& euler);
  void Euler2Quaternion(const Eigen::Vector3d& euler, Eigen::Quaterniond& quat);

  void Rotation_X(const double angle, Eigen::Matrix3d& rot_x);
  void Rotation_Y(const double angle, Eigen::Matrix3d& rot_y);
  void Rotation_Z(const double angle, Eigen::Matrix3d& rot_Z);

  void computeStateForTest(const std::vector<Eigen::Quaterniond>& p, const double t,
				       Eigen::Quaterniond& quat_t,
				       Eigen::Vector3d& w_t,
				       Eigen::Vector3d& dw_t);

private:

  double dt, t0, tf, threshold;
  double t_now, t0_updated;
  double ddN;
  Eigen::MatrixXd p_Linear = Eigen::MatrixXd(3, 6);
//   std::vector <Eigen::Vector3d> p_Linear;
  std::vector<Eigen::Quaterniond> p_Angular, p_Angular_old;
  Eigen::Quaterniond p_Angular0, p_Angular1, p_Angular2, p_Angular3, p_Angular4, p_Angular5;
//   Quaternions::Quaternion p_Angular0, p_Angular1, p_Angular2, p_Angular3, p_Angular4, p_Angular5;


  Eigen::Vector3d x0, dx0, ddx0, xf, dxf, ddxf, w0, dw0, wf, dwf;
  Eigen::Vector3d w_old, dw_old;
  Eigen::MatrixXd A_inv = Eigen::MatrixXd(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd(6);
  Eigen::VectorXd _p_Linear_i = Eigen::VectorXd(6);
  Eigen::Quaterniond quat0, quatf, p10_eigen, p1f_eigen, p20_eigen, p2f_eigen, p10quat0_eigen, p1_eigen, p2_eigen,
  p20quat0_eigen, p1fquatf_eigen, p2fquatf_eigen, mu, mu0, muf, dquatf, ddquatf, dquat0, ddquat0, Logq0invq1, dq_sr, ddq_sr,
  quat_nan, q_norm, dq_norm, ddq_norm;
  Eigen::Quaterniond quat_now, dquat_now, ddquat_now;
  Eigen::Quaterniond quat_old;
  Eigen::Quaterniond squad_a, squad_b;
  Eigen::Quaterniond w_quat, q_inverse;
  Eigen::Quaterniond q, dq, ddq;
//   Eigen::Vector4d quat0_v, quatf_v, p10, p1f, p20, p2f, p10quat0, p20quat0, p1fquatf, p2fquatf;
//     Quaternions::Quaternion quat0_v, quatf_v, p10, p1f, p20, p2f, p10quat0, p20quat0, p1fquatf, p2fquatf;


  //computeReferenceAcceleration
  Eigen::Vector3d x_des, dx_des, ddx_des, ddp_ref, w_des, dw_des, dw_ref;
  Eigen::Quaterniond quat, quat_des;
  double kp, kd, ka, ko;
  double t0_2, t0_3, t0_4, t0_5, tf_2, tf_3, tf_4, tf_5;

  bool traj_Angular_successive;

  std::string traj_type_Angular, method_Angular, traj_type_Linear;

  std::vector<Eigen::Quaterniond> quat_viapts, quat_cpoints, quat_all, quat_all_updated, squad_param_current;
  std::vector<double> t_viapts, t_all, t_all_updated;

  Eigen::Matrix2d m22;
  Eigen::Vector2d vec2, vec2_temp;

  Eigen::Matrix3d m33;
  Eigen::Vector3d vec3, vec3_temp;

  Eigen::VectorXd quat_elements_temp, v_all, cp, dp, v_temp;
  Eigen::Quaterniond omega_0_quat, omega_f_quat, q_temp1, q_temp2, q_temp3, term_log, term_exp1, term_exp2, exp1, exp2;
  Eigen::Quaterniond logq;
  Eigen::Quaterniond expq;
  Eigen::Quaterniond r1, r2;




  int ind_poly;

  size_t N, N_updated;

  struct Coeffs{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
  };

  Coeffs coeffs;

  std::vector<Coeffs> Vcoeffs;

  struct PosVel
  {
  double xi;
  double xii;
  double vi;
  double vii;
  };
  PosVel pv;

  struct VdV
  {
  double v0;
  double vN;
  double dv0;
  double dvN;
  };
  VdV vdv;

  struct ABCD
  {
    Eigen::VectorXd a;
    Eigen::VectorXd b;
    Eigen::VectorXd c;
    Eigen::VectorXd d;
    Eigen::VectorXd a_resized;
    Eigen::VectorXd c_resized;
  };
  ABCD abcd;

  struct QCoeffs{
    Eigen::Quaterniond p0;
    Eigen::Quaterniond p1;
    Eigen::Quaterniond p2;
    Eigen::Quaterniond p3;
    Eigen::Quaterniond p4;
  };

  QCoeffs qcoeffs, qcoeffs_old;

  struct dQddQ{
    Eigen::Quaterniond dq0;
    Eigen::Quaterniond dqf;
    Eigen::Quaterniond ddq0;
    Eigen::Quaterniond ddqf;
  };
  dQddQ dqddq;

  std::vector<QCoeffs> VQcoeffs, VQcoeffs_updated;


  void quintic_traj_eval3333(double t, Eigen::MatrixXd& p, Eigen::MatrixXd& des);

  void quintic_traj333 (const double t0, const double te, const Eigen::MatrixXd& X0,
		      Eigen::MatrixXd& Xe, Eigen::MatrixXd& p);

  void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error);

  void computeCoeffs_Linear ();
  void computeCoeffs_Angular(bool first_step = false);

  void computeQuaternionMultiplication(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
					Eigen::Quaterniond& q1q2);

  void computeQuaternionAndDerives(const double t, Eigen::Quaterniond& q,
							       Eigen::Quaterniond& dq,
					                       Eigen::Quaterniond& ddq);

  void Quaterniond2Vector4d(const Eigen::Quaterniond& quat, Eigen::Vector4d& vector);
//   void EigenQuat2QuaternionsQuat(const Eigen::Quaterniond& quat, Quaternions::Quaternion& quat2);

  void computeNDeriveParam(const std::vector<Eigen::Quaterniond>& p, const double t, double& DNperN, double& DDNperN);

  void computeQuinticQuatAndDerives(const std::vector<Eigen::Quaterniond>& p, const double t,
				    Eigen::Quaterniond& quat,
				    Eigen::Quaterniond& dquat,
				    Eigen::Quaterniond& ddquat);

  void computeQuinticDoubleAndDerives(const std::vector<double>& p, const double t,
						  double x,
						  double dx,
						  double ddx);

  void computeDDN(const double t, const double T, double& ddN);
  void transformPhase2Time(double tau, double& tau01, double& dtau01, double& ddtau01);
  void computeSlerp(double t,
		  Eigen::Quaterniond& s, Eigen::Quaterniond& ds, Eigen::Quaterniond& dds);

  void computeQuaternionLog(const Eigen::Quaterniond& q, Eigen::Quaterniond& logq);

  void solveLinearSys_abcd(const Eigen::VectorXd& a,
			 const Eigen::VectorXd& b,
                         const Eigen::VectorXd& c,
                         const Eigen::VectorXd& d,
                               Eigen::VectorXd& f);

  void computeQuaternionExp(const Eigen::Quaterniond& q, Eigen::Quaterniond& expq);

  void computeSquadICs(const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
                                   const Eigen::Vector3d omega_0, const Eigen::Vector3d omega_f,
				   const double T,
				   Eigen::Quaterniond& squad_a, Eigen::Quaterniond& squad_b);

  void computeSquadAux(double t, const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
					     const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
					     Eigen::Quaterniond& squad);
  void computeSquad(double t,
		  Eigen::Quaterniond& s, Eigen::Quaterniond& ds, Eigen::Quaterniond& dds);

  void computeSlerpSelf(const double t, const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
				    Eigen::Quaterniond& sl);

  void computeQuaternionPower(const Eigen::Quaterniond& q, const double t, Eigen::Quaterniond& q_power_t);

  void computeSquadSplineControlPoints();

  void getSquadSplineCurrentParams(const double t,
				 std::vector<Eigen::Quaterniond>& quat0,
				 double& t0_here, double& T_here);

  void computeDSquad_num(const double& t,
				     const Eigen::Quaterniond& q, const Eigen::Quaterniond& p,
				     const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
                                     Eigen::Quaterniond& ds);



//***NPoly and SRPoly***//

void computeCoeffs_first(const double& h, const double& dvi,
                                       Eigen::Vector2d& a3a4);

void computeCoeffs_last(const double& h, const double& dvii,
                                       Eigen::Vector3d& a2a3a4);

void computeCoeffs_general(const double& h, Eigen::Vector2d& a2a3);

void w2dq(const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d& dw,
                        Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq);

void computeCubicTraj(const QCoeffs& p, const double& tau,
                                    Eigen::Quaterniond& q, Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq);

void computeQuarticTraj(const QCoeffs& p, const double& tau,
                                    Eigen::Quaterniond& q, Eigen::Quaterniond& dq, Eigen::Quaterniond& ddq);

void computePolySplineCPointsAux(const std::vector<double>& t, const Eigen::VectorXd& x,
				 const bool& eval_A = true, const std::string& stype = "434");

void computePolySplineV(const std::vector<double>& t, const Eigen::VectorXd& x,
				 const bool& eval_A, const std::string& stype);

void computePolySplineCpoints(const std::vector<double>& t, const std::vector<Eigen::Quaterniond>& x, std::string stype);

void getPolySplineCurrentParams(const double& t, const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d dw,
				              QCoeffs& p, double& t0_here);

void ifSRPoly(const double& t, const Eigen::Quaterniond& q, const Eigen::Vector3d& w, const Eigen::Vector3d dw,
			    QCoeffs& p, double& t0_here);

void computeQuatNormDerivs(const QCoeffs& p, const double& t,
					 double& dN, double& ddN);

};

}





#endif // __TRAJ6D_H__
