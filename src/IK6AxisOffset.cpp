/*
 * IK6AxisOffset.cpp
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#include "/home/abhishek/eclipse-workspace/MotionPlanning/include/IK6AxisOffset.h"

namespace motion_planning {

IK6AxisOffset::IK6AxisOffset() {
	d1 = 0.15185;
	d4 = 0.13105;
	d5 = 0.08535;
	d6 = 0.0921;

	a2 = 0.24355;
	a3 = 0.2132;

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

    double th5_ratio = (des_pos(0)*sin(th1_1) - des_pos(1)*cos(th1_1) - d4)/d6;

    if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
      th5_ratio = th5_ratio/fabs(th5_ratio)*1;

    double th5_1 = acos(th5_ratio);
    double th5_2 = -th5_1;

    th5_ratio = (des_pos(0)*sin(th1_2) - des_pos(1)*cos(th1_2) - d4)/d6;

    if ( fabs( fabs(th5_ratio) - 1) < 0.0001)
      th5_ratio = th5_ratio/fabs(th5_ratio)*1;

    double th5_3 = acos(th5_ratio);
    double th5_4 = -th5_3;

    double th6_1, th6_2, th6_3, th6_4;

    if (fabs(sin(th5_1)) < 1e-5)
      th6_1 = 0;
    else
      th6_1 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_1), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_1));

    if (fabs(sin(th5_2)) < 1e-5)
      th6_2 = 0;
    else
      th6_2 = atan2((-eef_orient(0,1)*sin(th1_1) + eef_orient(1,1)*cos(th1_1))/sin(th5_2), -(-eef_orient(0,0)*sin(th1_1) + eef_orient(1,0)*cos(th1_1))/sin(th5_2));

    if (fabs(sin(th5_3)) < 1e-5)
      th6_3 = 0;
    else
      th6_3 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_3), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_3));

    if (fabs(sin(th5_4)) < 1e-5)
      th6_4 = 0;
    else
      th6_4 = atan2((-eef_orient(0,1)*sin(th1_2) + eef_orient(1,1)*cos(th1_2))/sin(th5_4), -(-eef_orient(0,0)*sin(th1_2) + eef_orient(1,0)*cos(th1_2))/sin(th5_4));

    std::vector<std::vector<double>> sol_init, sol_final;
    sol_init.resize(4);
    sol_final.resize(8);

    sol_init[0] = {th1_1, th5_1, th6_1};
    sol_init[1] = {th1_1, th5_2, th6_2};
    sol_init[2] = {th1_2, th5_3, th6_3};
    sol_init[3] = {th1_2, th5_4, th6_4};

    for (size_t ctr = 0; ctr < sol_init.size(); ctr++){

        double th1, th5, th6;

        th1 = sol_init[0][0];
        th5 = sol_init[0][1];
        th6 = sol_init[0][2];

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

        double th2_1 = atan2(y_13, x_13) - atan2(a3*sin(th3_1), (a2 + a3*cos(th3_1)));
        double th2_2 = atan2(y_13, x_13) - atan2(a3*sin(th3_2), (a2 + a3*cos(th3_2)));

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

        sol_final[2*ctr] = {th1, th2_1, th3_1, th4_1, th5, th6};
        sol_final[2*ctr+1] = {th1, th2_2, th3_2, th4_2, th5, th6};

    }

    return 0;

}

} /* namespace motion_planning */
