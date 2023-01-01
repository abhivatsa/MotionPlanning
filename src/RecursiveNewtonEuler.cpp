/*
 * RecursiveNewtonEuler.cpp
 *
 *  Created on: 31-Dec-2022
 *      Author: abhishek
 */

#include <MotionPlanning/RecursiveNewtonEuler.h>

namespace motion_planning {

RecursiveNewtonEuler::RecursiveNewtonEuler() {

	alpha.resize(6);
	a.resize(6);
	d.resize(6);
	theta.resize(6);
	m.resize(6);
	pos_com.resize(6);
	inertia_com.resize(6);

	double a2, a3, d1, d4, d5, d6;

	a2 = 0;
	a3 = 0;
	d1 = 0;
	d4 = 0;
	d5 = 0;
	d6 = 0;

	alpha = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};
	a = {0, 0, a2, a3, 0, 0};
	d = {d1, 0, 0, d4, d5, d6};
	theta = {0, 0, 0, 0, 0, 0};

	m = {1.98, 3.4445, 1.437, 0.871, 0.805, 0.261};

	pos_com[0] << 0, -0.02, 0;
	pos_com[1] << 0.13, 0, 0.1157;
	pos_com[2] << 0.05, 0, 0.0238;
	pos_com[3] << 0, 0, 0.01;
	pos_com[4] << 0, 0, 0.01;
	pos_com[5] << 0, 0, -0.02;

	inertia_com[0].setZero(3, 3);
	inertia_com[1].setZero(3, 3);
	inertia_com[2].setZero(3, 3);
	inertia_com[3].setZero(3, 3);
	inertia_com[4].setZero(3, 3);
	inertia_com[5].setZero(3, 3);

}

RecursiveNewtonEuler::~RecursiveNewtonEuler() {
	// TODO Auto-generated destructor stub
}

int RecursiveNewtonEuler::computeTorque(std::vector<double> joint_pos, std::vector<double> joint_vel, std::vector<double> joint_acc,
			std::vector<double>& joint_torque){

	Eigen::Matrix3d rotation_mat;
	Eigen::Vector3d pos_vec, z_dir;

	joint_torque.resize(joint_pos.size());

	z_dir << 0, 0, 1;

	std::vector<Eigen::Vector3d> force_com, torque_com;

	force_com[0].setZero(3);
	torque_com[0].setZero(3);

	Eigen::Vector3d omega, omega_next, omega_dot, omega_dot_next, vel_dot, vel_dot_next;

	omega.setZero(3);
	omega_dot.setZero(3);
	vel_dot.setZero(3);

	for (unsigned int joint_ctr = 0; joint_ctr < joint_pos.size(); joint_ctr++) {

		int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

		if (solve_chk == 0){

			omega_next = rotation_mat * omega + joint_vel[joint_ctr] * z_dir;
			omega_dot_next = rotation_mat * omega_dot + rotation_mat * ( omega.cross(z_dir) )
					+ joint_acc[joint_ctr] * z_dir;
			vel_dot_next = rotation_mat * ( omega_dot.cross(pos_vec) + omega.cross( omega.cross( pos_vec ) ) + vel_dot );

			Eigen::Vector3d vel_dot_com = omega_dot_next.cross(pos_com[joint_ctr]) + omega_next.cross( omega_next.cross( pos_com[joint_ctr]) ) + vel_dot;

			force_com[joint_ctr + 1] = m[joint_ctr]*vel_dot_com;
			torque_com[joint_ctr + 1] = inertia_com[joint_ctr]*omega_dot_next + omega_next.cross(inertia_com[joint_ctr]*omega_next);

			omega = omega_next;
			omega_dot = omega_dot_next;
			vel_dot = vel_dot_next;

		}
	}

	Eigen::Vector3d force, force_prev, torque, torque_prev;

	force_prev.setZero(3);
	torque_prev.setZero(3);

	for (unsigned int joint_ctr = joint_pos.size(); joint_ctr > 0; joint_ctr--){
		int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);

		if (solve_chk == 0){
			force = rotation_mat.transpose()*force_prev + force_com[joint_ctr];
			torque = torque_com[joint_ctr] + rotation_mat.transpose()*torque_prev
					+ pos_com[joint_ctr].cross(force_com[joint_ctr]) + pos_vec.cross(rotation_mat*force_prev);

			force_prev = force;
			torque_prev = torque;
		}

		joint_torque[joint_ctr - 1] = torque[2];

	}

	return 0;
}

int RecursiveNewtonEuler::computeTransformationMat(double joint_pos, int joint_num, Eigen::Matrix3d& rotation_mat,
			Eigen::Vector3d& pos_mat){

	rotation_mat.setIdentity(3, 3);
	pos_mat.setZero(3);

	rotation_mat << cos(joint_pos + theta[joint_num]), -sin(joint_pos + theta[joint_num]), 0,
					sin(joint_pos + theta[joint_num])*cos(alpha[joint_num]), cos(joint_pos + theta[joint_num])*cos(alpha[joint_num]), -sin(alpha[joint_num]),
					sin(joint_pos + theta[joint_num])*sin(alpha[joint_num]), cos(joint_pos + theta[joint_num])*sin(alpha[joint_num]), cos(alpha[joint_num]);

	pos_mat << a[joint_num], -sin(alpha[joint_num])*d[joint_num], cos(alpha[joint_num])*d[joint_num];

	return 0;

}

} /* namespace motion_planning */
