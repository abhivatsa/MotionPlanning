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

void ForwardKinematics::quaternionToEuler(const Eigen::Quaterniond &quat,
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

void ForwardKinematics::eulerToQuaternion(const double rpy[3],
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

int ForwardKinematics::computeFK(std::vector<double> joint_angles,
		double eef_pos[3], double eef_rpy[3]) {
	Eigen::MatrixXd trans_mat;
	computeFK(joint_angles, trans_mat);

	eef_pos[0] = trans_mat(0,3);
	eef_pos[1] = trans_mat(1,3);
	eef_pos[2] = trans_mat(2,3);

	Eigen::Matrix3d rot = trans_mat.topLeftCorner(3, 3);
	Eigen::Quaterniond quat(rot);
	quaternionToEuler(quat, eef_rpy);

	return 0;
}

} /* namespace motion_planning */
