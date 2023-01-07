/*
 * IK6AxisOffset.cpp
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#include <MotionPlanning/IK6AxisOffset.h>

namespace motion_planning {

IK6AxisOffset::IK6AxisOffset() {
	d1 = 0.15185;
	d4 = 0.13105;
	d5 = 0.08535;
	d6 = 0.0921;

	a2 = -0.24355;
	a3 = -0.2132;

}

IK6AxisOffset::~IK6AxisOffset() {
	// TODO Auto-generated destructor stub
}

int IK6AxisOffset::computeIK(Eigen::Vector3d eef_pos, Eigen::Matrix3d eef_orient,
		std::vector<double>& current_joint_val, std::vector<double>& joint_val){

	Eigen::Vector3d pos_wrist_eef, pose_base_shoulder, x_col, y_col, z_col, des_pos;

	x_col.setZero(3);
	y_col.setZero(3);
	z_col.setZero(3);
	des_pos.setZero();
	pos_wrist_eef.setZero(3);
	pose_base_shoulder.setZero(3);

	Eigen::MatrixXd T06;

	T06.setIdentity(4,4);

	T06.topLeftCorner(3,3) = eef_orient;
	T06.topRightCorner(3,1) = eef_pos;

	x_col(0) = 1.0;
	y_col(1) = 1.0;
	z_col(2) = 1.0;

	des_pos = eef_pos - d6*eef_orient*z_col;

	double R = sqrt(des_pos(0)*des_pos(0) + des_pos(1)*des_pos(1));

	double th1_1 = atan2(des_pos(1), des_pos(0)) + asin(d4/R);
	double th1_2 = atan2(des_pos(1), des_pos(0)) - asin(d4/R) + M_PI;

	if (fabs(th1_1) < 0.00001)
		th1_1 = 0;

	if (fabs(th1_2) < 0.00001)
		th1_2 = 0;

	double th1_3, th1_4;

	if (th1_1 > 0 && th1_1 < M_PI){
		th1_3 = - 2*M_PI + th1_1;
	}
	else{
		th1_3 = 2*M_PI + th1_1;
	}

	if (th1_2 > 0 && th1_2 < M_PI){
		th1_4 = - 2*M_PI + th1_2;
	}
	else{
		th1_4 = 2*M_PI + th1_2;
	}

	// computing th5 corresponding to th1_1

	double th5_ratio = (des_pos(0)*sin(th1_1) - des_pos(1)*cos(th1_1) - d4)/d6;

	if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
		th5_ratio = th5_ratio/fabs(th5_ratio)*1;

	double th5_1 = acos(th5_ratio);
	double th5_2 = -th5_1;

	double th5_5, th5_6;

	if (th5_1 > 0 && th5_1 < M_PI){
		th5_5 = - 2*M_PI + th5_1;
	}
	else{
		th5_5 = 2*M_PI + th5_1;
	}

	if (th5_2 > 0 && th5_2 < M_PI){
		th5_6 = - 2*M_PI + th5_2;
	}
	else{
		th5_6 = 2*M_PI + th5_2;
	}

	// computing th5 corresponding to th1_2

	th5_ratio = (des_pos(0)*sin(th1_2) - des_pos(1)*cos(th1_2) - d4)/d6;

	if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
		th5_ratio = th5_ratio/fabs(th5_ratio)*1;

	double th5_3 = acos(th5_ratio);
	double th5_4 = -th5_3;

	double th5_7, th5_8;

	if (th5_3 > 0 && th5_3 < M_PI){
		th5_7 = - 2*M_PI + th5_3;
	}
	else{
		th5_7 = 2*M_PI + th5_3;
	}

	if (th5_4 > 0 && th5_4 < M_PI){
		th5_8 = - 2*M_PI + th5_4;
	}
	else{
		th5_8 = 2*M_PI + th5_4;
	}

	// computing th5 corresponding to th1_3

	th5_ratio = (des_pos(0)*sin(th1_3) - des_pos(1)*cos(th1_3) - d4)/d6;

	if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
		th5_ratio = th5_ratio/fabs(th5_ratio)*1;

	double th5_9 = acos(th5_ratio);
	double th5_10 = -th5_3;

	double th5_11, th5_12;

	if (th5_9 > 0 && th5_9 < M_PI){
		th5_11 = - 2*M_PI + th5_9;
	}
	else{
		th5_11 = 2*M_PI + th5_9;
	}

	if (th5_10 > 0 && th5_10 < M_PI){
		th5_12 = - 2*M_PI + th5_10;
	}
	else{
		th5_12 = 2*M_PI + th5_10;
	}

	// computing th5 corresponding to th1_4

	th5_ratio = (des_pos(0)*sin(th1_4) - des_pos(1)*cos(th1_4) - d4)/d6;

	if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
		th5_ratio = th5_ratio/fabs(th5_ratio)*1;

	double th5_13 = acos(th5_ratio);
	double th5_14 = -th5_3;

	double th5_15, th5_16;

	if (th5_13 > 0 && th5_13 < M_PI){
		th5_15 = - 2*M_PI + th5_13;
	}
	else{
		th5_15 = 2*M_PI + th5_13;
	}

	if (th5_14 > 0 && th5_14 < M_PI){
		th5_16 = - 2*M_PI + th5_14;
	}
	else{
		th5_16 = 2*M_PI + th5_14;
	}


	double th6_1, th6_2, th6_3, th6_4, th6_5, th6_6, th6_7, th6_8, th6_9, th6_10, th6_11, th6_12, th6_13, th6_14, th6_15, th6_16;

	// Computing th6 for various value of th1_1 and th5_1, th5_2, th5_5, th5_6

	if (fabs(sin(th5_1)) < 1e-5)
		th6_1 = 0;
	else
		th6_1 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_1), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_1));

	if (fabs(sin(th5_2)) < 1e-5)
		th6_2 = 0;
	else
		th6_2 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_2), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_2));

	if (fabs(sin(th5_5)) < 1e-5)
		th6_5 = 0;
	else
		th6_2 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_5), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_5));

	if (fabs(sin(th5_6)) < 1e-5)
		th6_6 = 0;
	else
		th6_6 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_6), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_6));


	// Computing th6 for various value of th1_2 and th5_3, th5_4, th5_7, th5_8

	if (fabs(sin(th5_3)) < 1e-5)
		th6_3 = 0;
	else
		th6_3 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_3), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_3));

	if (fabs(sin(th5_4)) < 1e-5)
		th6_4 = 0;
	else
		th6_4 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_4), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_4));

	if (fabs(sin(th5_7)) < 1e-5)
		th6_7 = 0;
	else
		th6_7 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_7), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_7));

	if (fabs(sin(th5_8)) < 1e-5)
		th6_8 = 0;
	else
		th6_8 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_8), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_8));


	// Computing th6 for various value of th1_3 and th5_9, th5_10, th5_11, th5_12

	if (fabs(sin(th5_9)) < 1e-5)
		th6_9 = 0;
	else
		th6_9 = atan2((-eef_orient(0,1)*sin(th1_3) + eef_orient(1,1)*cos(th1_3))/sin(th5_9), -(-eef_orient(0,0)*sin(th1_3) + eef_orient(1,0)*cos(th1_3))/sin(th5_9));

	if (fabs(sin(th5_10)) < 1e-5)
		th6_10 = 0;
	else
		th6_10 = atan2((-eef_orient(0,1)*sin(th1_3) + eef_orient(1,1)*cos(th1_3))/sin(th5_10), -(-eef_orient(0,0)*sin(th1_3) + eef_orient(1,0)*cos(th1_3))/sin(th5_10));

	if (fabs(sin(th5_11)) < 1e-5)
		th6_11 = 0;
	else
		th6_11 = atan2((-eef_orient(0,1)*sin(th1_3) + eef_orient(1,1)*cos(th1_3))/sin(th5_11), -(-eef_orient(0,0)*sin(th1_3) + eef_orient(1,0)*cos(th1_3))/sin(th5_11));

	if (fabs(sin(th5_12)) < 1e-5)
		th6_12 = 0;
	else
		th6_12 = atan2((-eef_orient(0,1)*sin(th1_3) + eef_orient(1,1)*cos(th1_3))/sin(th5_12), -(-eef_orient(0,0)*sin(th1_3) + eef_orient(1,0)*cos(th1_3))/sin(th5_12));

	// Computing th6 for various value of th1_4 and th5_13, th5_14, th5_15, th5_16

	if (fabs(sin(th5_13)) < 1e-5)
		th6_13 = 0;
	else
		th6_13 = atan2((-eef_orient(0,1)*sin(th1_4) + eef_orient(1,1)*cos(th1_4))/sin(th5_13), -(-eef_orient(0,0)*sin(th1_4) + eef_orient(1,0)*cos(th1_4))/sin(th5_13));

	if (fabs(sin(th5_14)) < 1e-5)
		th6_14 = 0;
	else
		th6_14 = atan2((-eef_orient(0,1)*sin(th1_4) + eef_orient(1,1)*cos(th1_4))/sin(th5_14), -(-eef_orient(0,0)*sin(th1_4) + eef_orient(1,0)*cos(th1_4))/sin(th5_14));

	if (fabs(sin(th5_15)) < 1e-5)
		th6_15 = 0;
	else
		th6_15 = atan2((-eef_orient(0,1)*sin(th1_4) + eef_orient(1,1)*cos(th1_4))/sin(th5_15), -(-eef_orient(0,0)*sin(th1_4) + eef_orient(1,0)*cos(th1_4))/sin(th5_15));

	if (fabs(sin(th5_16)) < 1e-5)
		th6_16 = 0;
	else
		th6_16 = atan2((-eef_orient(0,1)*sin(th1_4) + eef_orient(1,1)*cos(th1_4))/sin(th5_16), -(-eef_orient(0,0)*sin(th1_4) + eef_orient(1,0)*cos(th1_4))/sin(th5_16));

	std::vector<std::vector<double>> sol_init, sol_final;
	sol_init.resize(16);
	sol_final.resize(128);

	sol_init[0] = {th1_1, th5_1, th6_1};
	sol_init[1] = {th1_1, th5_2, th6_2};
	sol_init[2] = {th1_1, th5_5, th6_5};
	sol_init[3] = {th1_1, th5_6, th6_6};

	sol_init[4] = {th1_2, th5_3, th6_3};
	sol_init[5] = {th1_2, th5_4, th6_4};
	sol_init[6] = {th1_2, th5_7, th6_7};
	sol_init[7] = {th1_2, th5_8, th6_8};

	sol_init[8] = {th1_3, th5_9, th6_9};
	sol_init[9] = {th1_3, th5_10, th6_10};
	sol_init[10] = {th1_3, th5_11, th6_11};
	sol_init[11] = {th1_3, th5_12, th6_12};

	sol_init[12] = {th1_4, th5_13, th6_13};
	sol_init[13] = {th1_4, th5_14, th6_14};
	sol_init[14] = {th1_4, th5_15, th6_15};
	sol_init[15] = {th1_4, th5_16, th6_16};

	for (int ctr = 0; ctr < 16; ctr++){
		std::cout<<"ctr "<<ctr<<" th1 : "<<sol_init[ctr][0]<<", th5:  "<<sol_init[ctr][1]<<", th6 : "<<sol_init[ctr][2]<<std::endl;
	}

	for (size_t ctr = 0; ctr < sol_init.size(); ctr++){

		double th1, th5, th6;

		th1 = sol_init[ctr][0];
		th5 = sol_init[ctr][1];
		th6 = sol_init[ctr][2];

		Eigen::MatrixXd T01, T45, T56, T14, T12, T23, T34;

		T01.resize(4,4);
		T45.resize(4,4);
		T56.resize(4,4);
		T14.resize(4,4);
		T12.resize(4,4);
		T23.resize(4,4);
		T34.resize(4,4);

		T01 << cos(th1), -sin(th1), 0, 0,
				sin(th1), cos(th1), 0, 0,
				0, 0, 1, d1,
				0, 0, 0, 1;

		T45 << cos(th5), -sin(th5), 0, 0,
				0, 0, -1, -d5,
				sin(th5), cos(th5), 0, 0,
				0, 0, 0, 1;

		T56 << cos(th6), -sin(th6), 0, 0,
				0, 0, 1, d6,
				-sin(th6), -cos(th6), 0, 0,
				0, 0, 0, 1;

		T14 = T01.inverse()*T06*T56.inverse()*T45.inverse();

		double x_13, y_13;

		x_13 = T14(0,3);
		y_13 = T14(2,3);

		double th3_ratio = (x_13*x_13 + y_13*y_13 - a2*a2 - a3*a3)/(2*a2*a3);

		if ( fabs( fabs(th3_ratio) - 1) < 0.0001 )
			th3_ratio = th3_ratio/fabs(th3_ratio)*1;

		double th3_1 = acos(th3_ratio);
		double th3_2 = -th3_1;

		double th3_3, th3_4;

		if (th3_1 > 0 && th3_1 < M_PI){
			th3_3 = - 2*M_PI + th3_1;
		}
		else{
			th3_3 = 2*M_PI + th3_1;
		}

		if (th3_2 > 0 && th3_2 < M_PI){
			th3_4 = - 2*M_PI + th3_2;
		}
		else{
			th3_4 = 2*M_PI + th3_2;
		}

		double th2_1 = atan2(y_13, x_13) - atan2(a3*sin(th3_1), (a2 + a3*cos(th3_1)));
		double th2_2 = atan2(y_13, x_13) - atan2(a3*sin(th3_2), (a2 + a3*cos(th3_2)));
		double th2_3 = atan2(y_13, x_13) - atan2(a3*sin(th3_3), (a2 + a3*cos(th3_3)));
		double th2_4 = atan2(y_13, x_13) - atan2(a3*sin(th3_4), (a2 + a3*cos(th3_4)));

		double th2_5, th2_6, th2_7, th2_8;

		// computing  th2_5 for th3_1

		if (th2_1 > 0 && th2_1 < M_PI){
			th2_5 = - 2*M_PI + th2_1;
		}
		else{
			th2_5 = 2*M_PI + th2_1;
		}

		// computing  th2_6 for th3_2

		if (th2_2 > 0 && th2_2 < M_PI){
			th2_6 = - 2*M_PI + th2_2;
		}
		else{
			th2_6 = 2*M_PI + th2_2;
		}

		// computing  th2_7 for th3_3

		if (th2_3 > 0 && th2_3 < M_PI){
			th2_7 = - 2*M_PI + th2_3;
		}
		else{
			th2_7 = 2*M_PI + th2_3;
		}

		// computing  th2_8 for th3_4

		if (th2_4 > 0 && th2_4 < M_PI){
			th2_8 = - 2*M_PI + th2_4;
		}
		else{
			th2_8 = 2*M_PI + th2_4;
		}

		// Computing th4_1 wrt th2_1 and th3_1

		T12 << cos(th2_1), -sin(th2_1), 0, 0,
				0, 0, -1, 0,
				sin(th2_1), cos(th2_1), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_1), -sin(th3_1), 0, a2,
				sin(th3_1), cos(th3_1), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_1 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_2 wrt th2_2 and th3_2

		T12 << cos(th2_2), -sin(th2_2), 0, 0,
				0, 0, -1, 0,
				sin(th2_2), cos(th2_2), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_2), -sin(th3_2), 0, a2,
				sin(th3_2), cos(th3_2), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_2 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_3 wrt th2_3 and th3_3

		T12 << cos(th2_3), -sin(th2_3), 0, 0,
				0, 0, -1, 0,
				sin(th2_3), cos(th2_3), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_3), -sin(th3_3), 0, a2,
				sin(th3_3), cos(th3_3), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_3 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_4 wrt th2_4 and th3_4

		T12 << cos(th2_4), -sin(th2_4), 0, 0,
				0, 0, -1, 0,
				sin(th2_4), cos(th2_4), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_4), -sin(th3_4), 0, a2,
				sin(th3_4), cos(th3_4), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_4 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_5 wrt th2_5 and th3_1

		T12 << cos(th2_5), -sin(th2_5), 0, 0,
				0, 0, -1, 0,
				sin(th2_5), cos(th2_5), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_1), -sin(th3_1), 0, a2,
				sin(th3_1), cos(th3_1), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_5 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_6 wrt th2_6 and th3_2

		T12 << cos(th2_6), -sin(th2_6), 0, 0,
				0, 0, -1, 0,
				sin(th2_6), cos(th2_6), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_2), -sin(th3_2), 0, a2,
				sin(th3_2), cos(th3_2), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_6 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_7 wrt th2_7 and th3_3

		T12 << cos(th2_7), -sin(th2_7), 0, 0,
				0, 0, -1, 0,
				sin(th2_7), cos(th2_7), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_3), -sin(th3_3), 0, a2,
				sin(th3_3), cos(th3_3), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_7 = atan2(-T34(1,2), T34(1,1));

		// Computing th4_8 wrt th2_8 and th3_4

		T12 << cos(th2_8), -sin(th2_8), 0, 0,
				0, 0, -1, 0,
				sin(th2_8), cos(th2_8), 0, 0,
				0, 0, 0, 1;

		T23 << cos(th3_4), -sin(th3_4), 0, a2,
				sin(th3_4), cos(th3_4), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		T34 = T23.inverse()*T12.inverse()*T14;

		double th4_8 = atan2(-T34(1,2), T34(1,1));

		sol_final[8*ctr] = {th1, th2_1, th3_1, th4_1, th5, th6};
		sol_final[8*ctr+1] = {th1, th2_5, th3_1, th4_5, th5, th6};
		sol_final[8*ctr+2] = {th1, th2_2, th3_2, th4_2, th5, th6};
		sol_final[8*ctr+3] = {th1, th2_6, th3_2, th4_6, th5, th6};
		sol_final[8*ctr+4] = {th1, th2_3, th3_3, th4_3, th5, th6};
		sol_final[8*ctr+5] = {th1, th2_7, th3_3, th4_7, th5, th6};
		sol_final[8*ctr+6] = {th1, th2_4, th3_4, th4_4, th5, th6};
		sol_final[8*ctr+7] = {th1, th2_8, th3_4, th4_8, th5, th6};

	}

	double least_sq_dist = std::numeric_limits<int>::max();

	joint_val = current_joint_val;

	std::cout<<"final soultion size : "<<sol_final.size()<<std::endl;

	for (size_t ctr = 0; ctr < sol_final.size(); ctr++){

		double sq_dist = 0;
		std::vector<double> local_sol = sol_final[ctr];

		for (size_t col_ctr =0; col_ctr < local_sol.size(); col_ctr++){
			sq_dist = sq_dist + (local_sol[col_ctr] - current_joint_val[col_ctr])*(local_sol[col_ctr] - current_joint_val[col_ctr]);
		}

		if (sq_dist < least_sq_dist){
			std::cout<<"sq_dist : "<<sq_dist<<std::endl;
			for (int count = 0; count < 6; count++){
				std::cout<<"local_sol : "<<count<<" : "<<local_sol[count]<<std::endl;
			}
			least_sq_dist = sq_dist;
			joint_val = local_sol;
		}

	}

	return 0;

}

} /* namespace motion_planning */
