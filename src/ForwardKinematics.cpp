/*
 * ForwardKinematics.cpp
 *
 *  Created on: 23-Dec-2022
 *      Author: abhishek
 */

#include "ForwardKinematics.h"

namespace motion_planning {

ForwardKinematics::ForwardKinematics() {
	alpha.resize(6);
	a.resize(6);
	d.resize(6);
	theta.resize(6);

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

ForwardKinematics::~ForwardKinematics() {
	// TODO Auto-generated destructor stub
}

int ForwardKinematics::getAlpha(std::vector<double>& alpha_vec){
	alpha_vec.resize(alpha.size());
	alpha_vec = alpha;
	return 0;
}

int ForwardKinematics::getA(std::vector<double>& a_vec){
	a_vec.resize(a.size());
	a_vec = a;
	return 0;
}

int ForwardKinematics::getD(std::vector<double>& d_vec){
	d_vec.resize(d.size());
	d_vec = d;
	return 0;
}

int ForwardKinematics::getTheta(std::vector<double>& theta_vec){
	theta_vec.resize(theta.size());
	theta_vec = theta;
	return 0;
}

int ForwardKinematics::computeFK(std::vector<double> joint_angles, Eigen::MatrixXd& trans_mat){

	Eigen::MatrixXd T_loc, T_glo;

	T_loc.resize(4,4);
	T_glo.resize(4,4);

	T_glo.setIdentity(4, 4);

	for (size_t ctr = 0; ctr < alpha.size(); ctr++){
		T_loc << cos(theta[ctr] + joint_angles[ctr]), -sin(theta[ctr] + joint_angles[ctr]), 0, a[0],
				sin(theta[ctr] + joint_angles[ctr])*cos(alpha[ctr]), cos(theta[ctr] + joint_angles[ctr])*cos(alpha[ctr]), -sin(alpha[ctr]), -d[ctr]*sin(alpha[ctr]),
				sin(theta[ctr] + joint_angles[ctr])*sin(alpha[ctr]), cos(theta[ctr] + joint_angles[ctr])*sin(alpha[ctr]), cos(alpha[ctr]), d[ctr]*cos(alpha[ctr]),
				0, 0, 0, 1;
		T_glo = T_glo * T_loc;
	}

	trans_mat.resize(4,4);

	trans_mat =  T_glo;

	return 0;


}

} /* namespace motion_planning */
