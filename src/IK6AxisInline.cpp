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

//	std::cout<<"eef_pos : \n"<<eef_pos<<std::endl;
//
//	std::cout<<"des_pos : \n"<<des_pos<<std::endl;

	if (fabs(des_pos(0)) < 1e-6){
		des_pos(0) = 0;
	}

	if (fabs(des_pos(1)) < 1e-6){
		des_pos(1) = 0;
	}

	if (fabs(des_pos(2)) < 1e-6){
		des_pos(2) = 0;
	}

//	std::cout<<"des_pos : \n"<<des_pos<<std::endl;

	double cos_th3 =  (des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) + des_pos(2)*des_pos(2) - l1*l1 - l2*l2)/(2*l1*l2);

	double diff_th3_sqrt_term = 1-cos_th3*cos_th3;

	if (fabs(diff_th3_sqrt_term) < 1e-6){
		diff_th3_sqrt_term = 0;
	}

	double th3_1 = atan2( sqrt(diff_th3_sqrt_term), cos_th3);
	double th3_2 = atan2( -sqrt(diff_th3_sqrt_term), cos_th3);

//	std::cout<<"des_pos : \n"<<des_pos<<std::endl;
//	std::cout<<"sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ) : "<<sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) )<<std::endl;
//	std::cout<<"l2*sin(th3_2) : "<<l2*sin(th3_2)<<", l1 + l2*cos(th3_2) : "<<l1 + l2*cos(th3_2)<<", atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) ) : "<<atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) )<<std::endl;

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

//	std::cout<<"th3_1 : "<<th3_1<<", th3_2 : "<<th3_2<<std::endl;
//	std::cout<<"th2_1 : "<<th2_1<<", th2_2 : "<<th2_2<<", th2_3 : "<<th2_3<<", th2_4 : "<<th2_4<<std::endl;

//	std::cout<<"th3_1 : "<<th3_1<<", th2_1 : "<<th2_1<<std::endl;
//	std::cout<<"th3_1 : "<<th3_1<<", th2_3 : "<<th2_3<<std::endl;
//	std::cout<<"th3_2 : "<<th3_2<<", th2_2 : "<<th2_2<<std::endl;
//	std::cout<<"th3_2 : "<<th3_2<<", th2_4 : "<<th2_4<<std::endl;
//
//	std::cout<<"first term  : "<<atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) )<<", 2nds term : "<<atan2( l2*sin(th3_1), ( l1 + l2*cos(th3_1) ) )<<std::endl;
//	std::cout<<"first term  : "<<atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), -des_pos(2) )<<", 2nds term : "<<atan2( l2*sin(th3_2), ( l1 + l2*cos(th3_2) ) )<<std::endl;
//	std::cout<<"first term  : "<<atan2( -sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) )<<", 2nds term new : "<<atan2( l2*sin(-th3_1), ( l1 + l2*cos(-th3_1) ) )<<std::endl;
//	std::cout<<"first term  : "<<atan2( -sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) )<<", 2nds term : "<<atan2( l2*sin(-th3_2), ( l1 + l2*cos(-th3_2) ) )<<std::endl;

	double th1_1;

//	std::cout<<"atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) ): "<<atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) )<<std::endl;

//	if (fabs(atan2( sqrt( des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1) ), des_pos(2) )) < M_PI/2){
//
//	}
//	else{
//		if ( atan2(des_pos(1), des_pos(0)) < 0){
//			th1_1 = atan2(des_pos(1), des_pos(0)) + M_PI;
//		}
//		else{
//			th1_1 = atan2(des_pos(1), des_pos(0)) - M_PI;
//		}
//	}

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

		rot_mat_03 << cos(th1)*cos(th2+th3), -sin(th1), cos(th1)*sin(th2+th3),
				sin(th1)*cos(th2+th3),  cos(th1), sin(th1)*sin(th2+th3),
				-sin(th2+th3), 0, cos(th2+th3);
//
		std::cout<<" ***************************************** "<<std::endl;
//
		std::cout<<"th1 : "<<th1<<", th2 : "<<th2<<", th3 : "<<th3<<std::endl;
//
		std::cout<<"rot_mat_03 : \n"<<rot_mat_03<<std::endl;
		std::cout<<"eef_orient : \n"<<eef_orient<<std::endl;

		Eigen::Matrix3d rot_mat_46;
		rot_mat_46 = (rot_mat_03.transpose())*eef_orient;

		std::cout<<"rot_mat_46 : \n"<<rot_mat_46<<std::endl;

		double th4_1, th5_1, th6_1, th4_2, th5_2, th6_2;

		th5_1 = atan2( sqrt( rot_mat_46(2,0)*rot_mat_46(2,0) + rot_mat_46(2,1)*rot_mat_46(2,1) ), rot_mat_46(2,2) );

		if (fabs(th5_1) < 1e-6){
			th4_1 = current_joint_val[3];
			th6_1 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) - th4_1;
		}
		else if (fabs(fabs(th5_1) - M_PI) < 1e-6){
			th4_1 = current_joint_val[3];
			th6_1 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) + th4_1;
		}
		else{
			th4_1 = atan2( rot_mat_46(1,2)/sin(th5_1), rot_mat_46(0,2)/sin(th5_1));
			th6_1 = atan2( rot_mat_46(2,1)/sin(th5_1), -rot_mat_46(2,0)/sin(th5_1));
		}

		th5_2 = atan2( -sqrt( rot_mat_46(2,0)*rot_mat_46(2,0) + rot_mat_46(2,1)*rot_mat_46(2,1) ), rot_mat_46(2,2) );

		if (fabs(th5_2) < 1e-6){
			th4_2 = current_joint_val[3];
			th6_2 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) - th4_2;
		}
		else if (fabs(fabs(th5_2) - M_PI) < 1e-6){
			th4_2 = current_joint_val[3];
			th6_2 = atan2(-rot_mat_46(0,1), rot_mat_46(0,0)) + th4_2;
		}
		else{
			th4_2 = atan2( rot_mat_46(1,2)/sin(th5_2), rot_mat_46(0,2)/sin(th5_2));
			th6_2 = atan2( rot_mat_46(2,1)/sin(th5_2), -rot_mat_46(2,0)/sin(th5_2));
		}

		std::cout<<"th4_1 : "<<th4_1<<", th5_1 : "<<th5_1<<", th6_1 : "<<th6_1<<std::endl;
		std::cout<<"th4_2 : "<<th4_2<<", th5_2 : "<<th5_2<<", th6_2 : "<<th6_2<<std::endl;

		std::vector<double> sol_1 = {th1, th2, th3, th4_1, th5_1, th6_1};
		std::vector<double> sol_2 = {th1, th2, th3, th4_2, th5_2, th6_2};

		pos_sol_final.push_back(sol_1);
		pos_sol_final.push_back(sol_2);

	}

//	for (int i = 0; i< pos_sol_final.size(); i++)
//	{
//		auto pos_ = pos_sol_final[i];
//
////		std::cout<<"i : "<<i<<" ("<<pos_[0]<<","<<pos_[1]<<","<<pos_[2]<<")\n";
//
//		double x = cos(pos_[0])*(l2*sin(pos_[1]+pos_[2]) + l1*cos(-M_PI/2+pos_[1]));
//		double y = sin(pos_[0])*(l2*sin(pos_[1]+pos_[2]) + l1*cos(-M_PI/2+pos_[1]));
//		double z = l2*cos(pos_[1]+pos_[2]) - l1*sin(-M_PI/2 + pos_[1]);
//
//		std::cout<<"("<<x<<","<<y<<","<<z<<")\n";
//	}

	double least_sq_dist = std::numeric_limits<int>::max();

	joint_val = current_joint_val;


	for (size_t ctr = 0; ctr < pos_sol_final.size(); ctr++){

		double sq_dist = 0;
		std::vector<double> local_sol = pos_sol_final[ctr];

		std::cout<<"j1 : "<<local_sol[0]<<", j2 : "<<local_sol[1]<<", j3 : "<<local_sol[2]<<", j4 : "<<local_sol[3]<<", j5 : "<<local_sol[4]<<", j6 : "<<local_sol[5]<<std::endl;


		for (size_t col_ctr =0; col_ctr < local_sol.size(); col_ctr++){
			sq_dist = sq_dist + (local_sol[col_ctr] - current_joint_val[col_ctr])*(local_sol[col_ctr] - current_joint_val[col_ctr]);
		}

		if (sq_dist < least_sq_dist){
			least_sq_dist = sq_dist;
//			std::cout<<"least_sq_dist: "<<least_sq_dist<<std::endl;
			joint_val = local_sol;
		}

	}

	return 0;

}

} /* namespace motion_planning */
