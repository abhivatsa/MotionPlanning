/*
 * Jacobian.cpp
 *
 *  Created on: 24-Dec-2022
 *      Author: abhishek
 */

#include <MotionPlanning/Jacobian.h>

namespace motion_planning {

Jacobian::Jacobian(int robot_type) : robot_type_(robot_type) {
	alpha.resize(6);
	a.resize(6);
	d.resize(6);
	theta.resize(6);

	if (robot_type_ == 0) // Offset type (Type B)
	{
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
	}
	else // Inline (Type A)
	{
		double a2, a3, d1, d2, d4, d5, d6;

		a2 = 0;
		a3 = 0;
		d1 = 0;
		d2 = 0;
		d4 = 0;
		d5 = 0;
		d6 = 0;

		alpha = {0, -M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2};
		a = {0, 0, a2, a3, 0, 0};
		d = {d1, d2, 0, d4, d5, d6};
		theta = {0, 0, 0, 0, 0, 0};
	}

}

Jacobian::~Jacobian() {
	// TODO Auto-generated destructor stub
}

int Jacobian::computeJacobian(std::vector<double> joint_angles, Eigen::MatrixXd& jacob_mat){

	if (robot_type_ == 0)
		computeJacobianOffset(joint_angles, jacob_mat);
	else
		computeJacobianInline(joint_angles, jacob_mat);

	return 0;
}

int Jacobian::computeJacobianOffset(std::vector<double> joint_angles, Eigen::MatrixXd& jacob_mat){

	Eigen::MatrixXd jacobian;
	jacobian.setIdentity(6, 6);

	double th1, th2, th3, th4, th5;

	th1 = joint_angles[0];
	th2 = joint_angles[1];
	th3 = joint_angles[2];
	th4 = joint_angles[3];
	th5 = joint_angles[4];

	jacobian(0,0) = -a[1]*sin(th1)*cos(th2) - a[2]*sin(th1)*cos(th2 + th3) + d[3]*cos(th1) - d[4]*sin(th1)*sin(th2 + th3 + th4) + d[5]*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5));
	jacobian(0,1) = -a[1]*sin(th2)*cos(th1) - a[2]*sin(th2 + th3)*cos(th1) + d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,2) = -a[2]*sin(th2 + th3)*cos(th1) + d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,3) =  d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,4) = d[5]*(-sin(th1)*sin(th5) - cos(th1)*cos(th5)*cos(th2 + th3 + th4));
	jacobian(0,5) = 0;

	jacobian(1,0) = a[1]*cos(th1)*cos(th2) + a[2]*cos(th1)*cos(th2 + th3) + d[3]*sin(th1) + d[4]*sin(th2 + th3 + th4)*cos(th1) + d[5]*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4));
	jacobian(1,1) = -a[1]*sin(th1)*sin(th2) - a[2]*sin(th1)*sin(th2 + th3) + d[4]*sin(th1)*cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)*sin(th2 + th3 + th4);
	jacobian(1,2) = -a[2]*sin(th1)*sin(th2 + th3) + d[4]*sin(th1)*cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)*sin(th2 + th3 + th4);
	jacobian(1,3) =  d[4]*sin(th1)*cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)*sin(th2 + th3 + th4);
	jacobian(1,4) = d[5]*(-sin(th1)*cos(th5)*cos(th2 + th3 + th4) + sin(th5)*cos(th1));
	jacobian(1,5) = 0;

	jacobian(2,0) = 0;
	jacobian(2,1) = a[1]*cos(th2) + a[2]*cos(th2 + th3) + d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4);
	jacobian(2,2) = a[2]*cos(th2 + th3) + d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4);
	jacobian(2,3) = d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4);
	jacobian(2,4) = -d[5]*sin(th2 + th3 + th4)*cos(th5);
	jacobian(2,5) = 0;

	jacobian(3,0) = 0;
	jacobian(3,1) = sin(th1);
	jacobian(3,2) = sin(th1);
	jacobian(3,3) = sin(th1);
	jacobian(3,4) = cos(th1)*sin(th2+th3+th4);
	jacobian(3,5) = -cos(th1)*cos(th2+th3+th4)*sin(th5) + sin(th1)*cos(th5);

	jacobian(4,0) = 0;
	jacobian(4,1) = -cos(th1);
	jacobian(4,2) = -cos(th1);
	jacobian(4,3) = -cos(th1);
	jacobian(4,4) = sin(th1)*sin(th2+th3+th4);
	jacobian(4,5) = -sin(th1)*cos(th2+th3+th4)*sin(th5) - cos(th1)*cos(th5);

	jacobian(5,0) = 1;
	jacobian(5,1) = 0;
	jacobian(5,2) = 0;
	jacobian(5,3) = 0;
	jacobian(5,4) = -cos(th2+th3+th4);
	jacobian(5,5) = -sin(th2+th3+th4)*sin(th5);


	jacob_mat = jacobian;

	return 0;
}


int Jacobian::computeJacobianInline(std::vector<double> joint_angles, Eigen::MatrixXd& jacob_mat){
	Eigen::MatrixXd jacobian;
	jacobian.setIdentity(6, 6);

	double th1, th2, th3, th4, th5;

	th1 = joint_angles[0];
	th2 = joint_angles[1];
	th3 = joint_angles[2];
	th4 = joint_angles[3];
	th5 = joint_angles[4];

	jacobian(0,0) = -a[1]*sin(th1)*cos(th2) -a[2]*sin(th1)*cos(th2+th3) -d[1]*cos(th1) -d[3]*sin(th1)*sin(th2+th3) -d[4]*( cos(th1)*cos(th4) - sin(th1)*sin(th4)*cos(th2-th3) ) -d[5]*( sin(th1)*sin(th5)*cos(th4)*cos(th2+th3) + sin(th1)*cos(th5)*sin(th2+th3) + cos(th1)*sin(th4)*sin(th5) );
	jacobian(0,1) = -a[1]*cos(th1)*sin(th2) - a[2]*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - d[3]*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + d[4]*sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))  -d[5]*(cos(th5)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th4)*sin(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)));
	jacobian(0,2) = d[3]*cos(th2 + th3) - a[2]*sin(th2 + th3) + d[5]*cos(th2 + th3)*cos(th5) + d[4]*sin(th2 + th3)*sin(th4) - d[5]*sin(th2 + th3)*cos(th4)*sin(th5);
	jacobian(0,3) = d[4]*(sin(th1)*sin(th4) + cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - d[5]*sin(th5)*(cos(th4)*sin(th1) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)));
	jacobian(0,4) = -d[5]*(cos(th5)*(sin(th1)*sin(th4) + cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + sin(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)));
	jacobian(0,5) = 0;

	jacobian(1,0) = d[3]*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - a[2]*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - d[5]*(sin(th5)*(sin(th1)*sin(th4) + cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - cos(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))) - d[1]*sin(th1) - d[4]*(cos(th4)*sin(th1) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a[1]*cos(th1)*cos(th2);
	jacobian(1,1) = d[4]*sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - d[3]*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) - d[5]*(cos(th5)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))) - a[2]*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - a[1]*sin(th1)*sin(th2);
	jacobian(1,2) = sin(th1)*(d[3]*cos(th2 + th3) - a[2]*sin(th2 + th3) + d[5]*cos(th2 + th3)*cos(th5) + d[4]*sin(th2 + th3)*sin(th4) - d[5]*sin(th2 + th3)*cos(th4)*sin(th5));
	jacobian(1,3) = d[5]*sin(th5)*(cos(th1)*cos(th4) + sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) - d[4]*(cos(th1)*sin(th4) - cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)));
	jacobian(1,4) = d[6]*(cos(th5)*(cos(th1)*sin(th4) - cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) - sin(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)));
	jacobian(1,5) = 0;

	jacobian(2,0) = 0;
	jacobian(2,1) = d[4]*cos(th2 + th3)*sin(th4) - d[3]*sin(th2 + th3) - a[1]*cos(th2) - (d[5]*cos(th2 + th3)*sin(th4 + th5))/2 - a[2]*cos(th2 + th3) - d[5]*sin(th2 + th3)*cos(th5) + (d[5]*sin(th4 - th5)*cos(th2 + th3))/2;
	jacobian(2,2) = d[4]*cos(th2 + th3)*sin(th4) - d[3]*sin(th2 + th3) - a[2]*cos(th2 + th3) - d[5]*sin(th2 + th3)*cos(th5) - d[5]*cos(th2 + th3)*cos(th4)*sin(th5);
	jacobian(2,3) = sin(th2 + th3)*(d[4]*cos(th4) + d[5]*sin(th4)*sin(th5));
	jacobian(2,4) = - d[5]*cos(th2 + th3)*sin(th5) - d[5]*sin(th2 + th3)*cos(th4)*cos(th5);
	jacobian(2,5) = 0;

	jacobian(3,0) = 0;
	jacobian(3,1) = -sin(th1);
	jacobian(3,2) = -sin(th1);
	jacobian(3,3) = cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2);
	jacobian(3,4) = sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th4)*sin(th1);
	jacobian(3,5) = cos(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th5)*(sin(th1)*sin(th4) + cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)));

	jacobian(4,0) = 0;
	jacobian(4,1) = cos(th1);
	jacobian(4,2) = cos(th1);
	jacobian(4,3) = cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2);
	jacobian(4,4) = cos(th1)*cos(th4) + sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1));
	jacobian(4,5) = sin(th5)*(cos(th1)*sin(th4) - cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + cos(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2));

	jacobian(5,0) = 1;
	jacobian(5,1) = 0;
	jacobian(5,2) = 0;
	jacobian(5,3) = cos(th2)*cos(th3) - sin(th2)*sin(th3);
	jacobian(5,4) = sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2));
	jacobian(5,5) = cos(th5)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - cos(th4)*sin(th5)*(cos(th2)*sin(th3) + cos(th3)*sin(th2));

	jacob_mat = jacobian;

	return 0;
}

} /* namespace motion_planning */
