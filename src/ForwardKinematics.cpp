/*
 * ForwardKinematics.cpp
 *
 *  Created on: 23-Dec-2022
 *      Author: abhishek
 */

#include <MotionPlanning/ForwardKinematics.h>

namespace motion_planning {

ForwardKinematics::ForwardKinematics() {
	alpha.resize(6);
	a.resize(6);
	d.resize(6);
	theta.resize(6);

	double a2, a3, d1, d4, d5, d6;

	/* Offset */
//	a2 = -0.24355;
//	a3 = -0.2132;
//	d1 = 0.15185;
//	d4 = 0.13105;
//	d5 = 0.08535;
//	d6 = 0.0921;
//
//	alpha = {0, M_PI/2, 0, 0, M_PI/2, -M_PI/2};
//	a = {0, 0, a2, a3, 0, 0};
//	d = {d1, 0, 0, d4, d5, d6};
//	theta = {0, 0, 0, 0, 0, 0};



/* Inline */
	alpha = {0, -M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2};
	a = {0, 0, 0.6, 0, 0, 0};
	d = {0, 0, 0, 0.4, 0, 0};
	theta = {0, -M_PI/2, M_PI/2, 0, 0, 0};

}

ForwardKinematics::~ForwardKinematics() {
	// TODO Auto-generated destructor stub
}

int ForwardKinematics::computeFK(std::vector<double> joint_angles, Eigen::MatrixXd& trans_mat){

	Eigen::MatrixXd T_loc, T_glo;

	T_loc.resize(4,4);
	T_glo.resize(4,4);


	T_glo.setIdentity(4, 4);

	for (size_t ctr = 0; ctr < alpha.size(); ctr++){

		T_loc << cos(theta[ctr] + joint_angles[ctr]), -sin(theta[ctr] + joint_angles[ctr]), 0, a[ctr],
				sin(theta[ctr] + joint_angles[ctr])*cos(alpha[ctr]), cos(theta[ctr] + joint_angles[ctr])*cos(alpha[ctr]), -sin(alpha[ctr]), -d[ctr]*sin(alpha[ctr]),
				sin(theta[ctr] + joint_angles[ctr])*sin(alpha[ctr]), cos(theta[ctr] + joint_angles[ctr])*sin(alpha[ctr]), cos(alpha[ctr]), d[ctr]*cos(alpha[ctr]),
				0, 0, 0, 1;

		std::cout<<" ***************************************** FK : "<<ctr<<std::endl;

		std::cout<<"T_loc : \n"<<T_loc<<std::endl;


		T_glo = T_glo * T_loc;
		std::cout<<"T_glo : \n"<<T_glo<<std::endl;

	}

	trans_mat.resize(4,4);

	trans_mat =  T_glo;

	return 0;


}

} /* namespace motion_planning */
