/*
 * IK6AxisInline.cpp
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#include "IK6AxisInline.h"

namespace motion_planning {

IK6AxisInline::IK6AxisInline() {
	l0 = 360;
	l1 = 400;
	l2 = 400;
	l3 = 80;

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

	double th3_1 = acos((des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) + des_pos(2)*des_pos(2) - l1*l1 - l2*l2)/(2*l1*l2));
	double th3_2 = - th3_1;

	double th2_1 = atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_1), ( l1 + l2*cos(th3_1) ) );
	double th2_2 = atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ) - atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) );

	double th1_1 = atan(des_pos(1) / des_pos(0));
	double th1_2 = 0;

	if (th1_1 < 0){
		th1_2 = th1_1 + M_PI;
	}else{
		th1_2 = th1_1 - M_PI;
	}

	// std::cout<<"th3_1 : "<<th3_1<<", th3_2 : "<<th3_2<<", th2_1 : "<<th2_1<<", th2_2 : "<<th2_2<<", th1_1 : "<<th1_1<<", th1_2 : "<<th1_2<<std::endl;

	std::vector<std::vector<double>> pos_sol_final;

	for (int ctr = 0; ctr < 3; ctr++){

		double th1, th2, th3;

		if (ctr == 0){
			th1 = th1_1; th2 = th2_1; th3 = th3_1;
		}
		else if (ctr == 1){
			th1 = th1_2; th2 = th2_1; th3 = th3_1;
		}
		else if (ctr == 2){
			th1 = th1_1; th2 = th2_2; th3 = th3_2;
		}
		else{
			th1 = th1_2; th2 = th2_2; th3 = th3_2;
		}

		Eigen::Matrix3d rot_mat_03;

		rot_mat_03 << cos(th1)*cos(th2+th3), -sin(th1), cos(th1)*sin(th2+th3),
					sin(th1)*cos(th2+th3),  cos(th1), sin(th1)*sin(th2+th3),
					-sin(th2+th3), 0, cos(th2+th3);

		Eigen::Matrix3d rot_mat_46 = rot_mat_03.transpose()*eef_orient;

		double th4_1, th5_1, th6_1, th4_2, th5_2, th6_2;

		th5_1 = atan2( sqrt( rot_mat_46(2,0)*rot_mat_46(2,0) + rot_mat_46(2,1)*rot_mat_46(2,1) ), rot_mat_46(2,2) );
		th4_1 = atan2( rot_mat_46(1,2)/sin(th5_1), rot_mat_46(0,2)/sin(th5_1));
		th6_1 = atan2( rot_mat_46(2,1)/sin(th5_1), -rot_mat_46(2,0)/sin(th5_1));

		th5_2 = atan2( -sqrt( rot_mat_46(2,0)*rot_mat_46(2,0) + rot_mat_46(2,1)*rot_mat_46(2,1) ), rot_mat_46(2,2) );
		th4_2 = atan2( rot_mat_46(1,2)/sin(th5_2), rot_mat_46(0,2)/sin(th5_2));
		th6_2 = atan2( rot_mat_46(2,1)/sin(th5_2), -rot_mat_46(2,0)/sin(th5_2));

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

} /* namespace motion_planning */
