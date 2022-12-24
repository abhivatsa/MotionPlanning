/*
 * Jacobian.cpp
 *
 *  Created on: 24-Dec-2022
 *      Author: abhishek
 */

#include "/home/abhishek/eclipse-workspace/MotionPlanning/include/Jacobian.h"

namespace motion_planning {

Jacobian::Jacobian() {
	// TODO Auto-generated constructor stub

}

Jacobian::~Jacobian() {
	// TODO Auto-generated destructor stub
}

int Jacobian::computeJacobian(std::vector<double> joint_angles, Eigen::MatrixXd& jacob_mat){

	Eigen::MatrixXd jacobian;
	jacobian.setIdentity(6, 6);

	double th1, th2, th3, th4, th5, th6;

	th1 = joint_angles[0];
	th2 = joint_angles[1];
	th3 = joint_angles[2];
	th4 = joint_angles[3];
	th5 = joint_angles[4];
	th6 = joint_angles[5];

	jacobian(0,0) = -a[1]*sin(th1)*cos(th2) - a[2]*sin(th1)*cos(th2 + th3) + d[3]*cos(th1) - d[4]*sin(th1)*sin(th2 + th3 + th4) + d[5]*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5));
	jacobian(0,1) = -a[1]*sin(th2)*cos(th1) - a[2]*sin(th2 + th3)*cos(th1) + d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,2) = -a[2]*sin(th2 + th3)*cos(th1) + d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,3) =  d[4]*cos(th1)*cos(th2 + th3 + th4) + d[5]*sin(th5)*sin(th2 + th3 + th4)*cos(th1);
	jacobian(0,4) = d[5]*(-sin(th1)*sin(th5) - cos(th1)*cos(th5)*cos(th2 + th3 + th4));
	jacobian(0,5) = 0;

	jacobian(1,0) = a[1]*cos(th1)*cos(th2) + a[2]*cos(th1)*cos(th2 + th3) - d[3]*cos(th1) + d[4]*sin(th2 + th3 + th4)*cos(th1) + d[5]*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4));
	jacobian(1,1) = -a[1]*sin(th1)*sin(th2) - a[2]*sin(th1)*sin(th2 + th3) + d[4]*sin(th1)*cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)*sin(th2 + th3 + th4);
	jacobian(1,2) = -a[2]*sin(th1)*sin(th2 + th3) + d[4]*sin(th1)⋅cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)⋅sin(th2 + th3 + th4);
	jacobian(1,3) =  d[4]*sin(th1)*cos(th2 + th3 + th4) + d[5]*sin(th1)*sin(th5)*sin(th2 + th3 + th4);
	jacobian(1,4) = d[5]*(-sin(th1)*cos(th5)*cos(th2 + th3 + th4) + sin(th5)*cos(th1));
	jacobian(1,5) = 0;

	jacobian(2,0) = 0;
	jacobian(2,1) = a[1]*cos(th2) + a[2]*cos(th2 + th3) + d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4);
	jacobian(2,2) = a[2]*cos(th2 + th3) + d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4);
	jacobian(2,3) = d[4]*sin(th2 + th3 + th4) - d[5]*sin(th5)*cos(th2 + th3 + th4)
	jacobian(2,4) = -d[5]*sin(th2 + th3 + th4)*cos(th5)
	jacobian(2,5) = 0;

	return 0;
}

} /* namespace motion_planning */
