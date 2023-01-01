/*
 * RecursiveNewtonEuler.h
 *
 *  Created on: 31-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_MOTIONPLANNING_RECURSIVENEWTONEULER_H_
#define INCLUDE_MOTIONPLANNING_RECURSIVENEWTONEULER_H_

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace motion_planning {

class RecursiveNewtonEuler {
public:
	RecursiveNewtonEuler();
	virtual ~RecursiveNewtonEuler();
	int computeTorque(std::vector<double> joint_pos, std::vector<double> joint_vel, std::vector<double> joint_acc,
			std::vector<double>& joint_torque);
	int computeTransformationMat(double joint_pos, int joint_num, Eigen::Matrix3d& rotation_mat,
			Eigen::Vector3d& pos_mat);
private:
	std::vector<double> alpha, a, d, theta, m;
	std::vector<Eigen::Vector3d> pos_com;
	std::vector<Eigen::Matrix3d> inertia_com;

};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_RECURSIVENEWTONEULER_H_ */
