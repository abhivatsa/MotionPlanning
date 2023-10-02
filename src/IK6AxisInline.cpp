/*
 * IK6AxisInline.cpp
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#include <MotionPlanning/IK6AxisInline.h>

namespace motion_planning {

IK6AxisInline::IK6AxisInline() {
//	l0 = 360;
//	l1 = 400;
//	l2 = 400;
//	l3 = 80;
	l0 = 0;
	l1 = 0.6;
	l2 = 0.4;
	l3 = 0;

}

IK6AxisInline::~IK6AxisInline() {
	// TODO Auto-generated destructor stub
}

int IK6AxisInline::computeIK(Eigen::Vector3d eef_pos, Eigen::Matrix3d eef_orient,
		std::vector<double>& current_joint_val, std::vector<double>& joint_val){

	Eigen::Vector3d pos_wrist_eef, pose_base_shoulder;

	pos_wrist_eef.setZero(3);
	pose_base_shoulder.setZero(3);

	pos_wrist_eef(2) = l3;
	pose_base_shoulder(2) = l0;
	Eigen::Vector3d des_pos = eef_pos - eef_orient*pos_wrist_eef - pose_base_shoulder;

	if (fabs(des_pos(0)) < 1e-6){
		des_pos(0) = 0;
	}

	if (fabs(des_pos(1)) < 1e-6){
		des_pos(1) = 0;
	}

	if (fabs(des_pos(2)) < 1e-6){
		des_pos(2) = 0;
	}

	double cos_th3 =  (des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) + des_pos(2)*des_pos(2) - l1*l1 - l2*l2)/(2*l1*l2);

	double diff_th3_sqrt_term = 1-cos_th3*cos_th3;

	if (fabs(diff_th3_sqrt_term) < 1e-6){
		diff_th3_sqrt_term = 0;
	}

	double th3_1 = atan2( sqrt(diff_th3_sqrt_term), cos_th3);
	double th3_2 = atan2( -sqrt(diff_th3_sqrt_term), cos_th3);

	double th2_1 = atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_1), ( l1 + l2*cos(th3_1) ) );
	double th2_2 = atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) );
	double th2_3 = atan2( -sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_1), ( l1 + l2*cos(th3_1) ) );
	double th2_4 = atan2( -sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) );

	if (fabs(th2_1) > M_PI){
		if (th2_1 > M_PI){
			th2_1 = th2_1 - 2*M_PI;
		}
		else{
			th2_1 = th2_1 + 2*M_PI;
		}
	}

	if (fabs(th2_2) > M_PI){
		if (th2_2 > M_PI){
			th2_2 = th2_2 - 2*M_PI;
		}
		else{
			th2_2 = th2_2 + 2*M_PI;
		}
	}

	if (fabs(th2_3) > M_PI){
		if (th2_3 > M_PI){
			th2_3 = th2_3 - 2*M_PI;
		}
		else{
			th2_3 = th2_3 + 2*M_PI;
		}
	}

	if (fabs(th2_4) > M_PI){
		if (th2_4 > M_PI){
			th2_4 = th2_4 - 2*M_PI;
		}
		else{
			th2_4 = th2_4 + 2*M_PI;
		}
	}

	double th1_1;

	th1_1 = atan2(des_pos(1), des_pos(0));

	double th1_2 = 0;

	if (th1_1 < 0){
		th1_2 = th1_1 + M_PI;
	}else{
		th1_2 = th1_1 - M_PI;
	}

	std::vector<std::vector<double>> pos_sol_final;

	for (int ctr = 0; ctr < 4; ctr++){

		double th1, th2, th3;

		if (ctr == 0){
			th1 = th1_1; th2 = th2_1; th3 = th3_1;
		}
		else if (ctr == 1){
			th1 = th1_1; th2 = th2_2; th3 = th3_2;
		}
		else if (ctr == 2){
			th1 = th1_2; th2 = th2_3; th3 = th3_1;
		}
		else{
			th1 = th1_2; th2 = th2_4; th3 = th3_2;
		}

		Eigen::Matrix3d rot_mat_03;

		rot_mat_03 << cos(th1)*cos(th2+th3), -cos(th1)*sin(th2+th3), -sin(th1),
				sin(th1)*cos(th2+th3), -sin(th1)*sin(th2+th3), cos(th1),
				-sin(th2+th3), -cos(th2+th3), 0;

		Eigen::Matrix3d rot_mat_46;
		rot_mat_46 = (rot_mat_03.transpose())*eef_orient;

		double th4_1, th5_1, th6_1, th4_2, th5_2, th6_2;

		th5_1 = atan2( sqrt( rot_mat_46(1,0)*rot_mat_46(1,0) + rot_mat_46(1,1)*rot_mat_46(1,1) ), -rot_mat_46(1,2) );

		if (fabs(th5_1) < 1e-6){
			th4_1 = current_joint_val[3];
//			th6_1 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) - th4_1;
			th6_1 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) - th4_1;
		}
		else if (fabs(fabs(th5_1) - M_PI) < 1e-6){
			th4_1 = current_joint_val[3];
			th6_1 = atan2(rot_mat_46(0,1), -rot_mat_46(0,0)) + th4_1;

			if (fabs(th6_1) > M_PI){
				if (th6_1 > M_PI){
					th6_1 = th6_1 - 2*M_PI;
				}
				else{
					th6_1 = th6_1 + 2*M_PI;
				}
			}
		}
		else{
			th4_1 = atan2( rot_mat_46(2,2)/sin(th5_1), rot_mat_46(0,2)/sin(th5_1));
			th6_1 = atan2( -rot_mat_46(1,1)/sin(th5_1), rot_mat_46(1,0)/sin(th5_1));
		}

		th5_2 = atan2( -sqrt( rot_mat_46(1,0)*rot_mat_46(1,0) + rot_mat_46(1,1)*rot_mat_46(1,1) ), -rot_mat_46(1,2) );

		if (fabs(th5_2) < 1e-6){
			th4_2 = current_joint_val[3];
			th6_2 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) - th4_2;
		}
		else if (fabs(fabs(th5_2) - M_PI) < 1e-6){
			th4_2 = current_joint_val[3];
			th6_2 = atan2(rot_mat_46(0,1), -rot_mat_46(0,0)) + th4_2;
			if (fabs(th6_2) > M_PI){
				if (th6_2 > M_PI){
					th6_2 = th6_2 - 2*M_PI;
				}
				else{
					th6_2 = th6_2 + 2*M_PI;
				}
			}
		}
		else{
			th4_2 = atan2( rot_mat_46(2,2)/sin(th5_2), rot_mat_46(0,2)/sin(th5_2));
			th6_2 = atan2( -rot_mat_46(1,1)/sin(th5_2), rot_mat_46(1,0)/sin(th5_2));
		}

		std::vector<double> sol_1 = {th1, th2, th3, th4_1, th5_1, th6_1};
		std::vector<double> sol_2 = {th1, th2, th3, th4_2, th5_2, th6_2};

		pos_sol_final.push_back(sol_1);
		pos_sol_final.push_back(sol_2);

	}

	double least_sq_dist = std::numeric_limits<int>::max();

	joint_val = current_joint_val;


	for (size_t ctr = 0; ctr < pos_sol_final.size(); ctr++){

		double sq_dist = 0;
		std::vector<double> local_sol = pos_sol_final[ctr];

		for (size_t col_ctr =0; col_ctr < local_sol.size(); col_ctr++){
			sq_dist = sq_dist + (local_sol[col_ctr] - current_joint_val[col_ctr])*(local_sol[col_ctr] - current_joint_val[col_ctr]);
		}

		if (sq_dist < least_sq_dist){
			least_sq_dist = sq_dist;
			joint_val = local_sol;
		}

	}

	return 0;

}

int IK6AxisInline::computeIK(const std::array<double, 3> &eef_pos,
		const std::array<double, 3> &eef_rpy,
		const std::array<double, 6> &current_joint_position,
		std::array<double, 6> &joint_val) {

	Eigen::Vector3d pos(eef_pos[0], eef_pos[1], eef_pos[2]);
	Eigen::Quaterniond quat;
	double rpy[3] = {eef_rpy[0], eef_rpy[1], eef_rpy[2]};
	eulerToQuaternion(rpy, quat);

	Eigen::Matrix3d rot(quat);

	std::vector<double> current_position, solution;
	for(int i = 0; i < 6; i++)
		current_position.push_back(current_joint_position[i]);

	computeIK(pos, rot, current_position, solution);

	for(int i = 0; i < 6; i++)
		joint_val[i] = solution[i];

	return 0;
}

int IK6AxisInline::computeIK(const double eef_pos[3], const double eef_rpy[3],
		const double current_joint_position[6], double joint_val[6]) {

	Eigen::Vector3d pos(eef_pos[0], eef_pos[1], eef_pos[2]);
	Eigen::Quaterniond quat;

	eulerToQuaternion(eef_rpy, quat);

	Eigen::Matrix3d rot(quat);

	std::vector<double> current_position, solution;
	for(int i = 0; i < 6; i++)
		current_position.push_back(current_joint_position[i]);

	computeIK(pos, rot, current_position, solution);

	for(int i = 0; i < 6; i++)
		joint_val[i] = solution[i];

	return 0;
}

void IK6AxisInline::quaternionToEuler(const Eigen::Quaterniond &quat,
		double rpy[3]) {

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
	double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
	rpy[0] = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = sqrt(1 + 2 * (quat.w() * quat.y() - quat.x() * quat.z()));
	double cosp = sqrt(1 - 2 * (quat.w() * quat.y() - quat.x() * quat.z()));
	rpy[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
	double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
	rpy[2] = atan2(siny_cosp, cosy_cosp);
}

void IK6AxisInline::eulerToQuaternion(const double rpy[3],
		Eigen::Quaterniond &quat) {
	double cr = cos(rpy[0] * 0.5);
	double sr = sin(rpy[0] * 0.5);
	double cp = cos(rpy[1] * 0.5);
	double sp = sin(rpy[1] * 0.5);
	double cy = cos(rpy[2] * 0.5);
	double sy = sin(rpy[2] * 0.5);

	quat.w() = cr * cp * cy + sr * sp * sy;
	quat.x() = sr * cp * cy - cr * sp * sy;
	quat.y() = cr * sp * cy + sr * cp * sy;
	quat.z() = cr * cp * sy - sr * sp * cy;
}

} /* namespace motion_planning */
