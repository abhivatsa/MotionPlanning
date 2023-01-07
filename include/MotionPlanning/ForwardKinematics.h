/*
 * ForwardKinematics.h
 *
 *  Created on: 23-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_MOTIONPLANNING_FORWARDKINEMATICS_H_
#define INCLUDE_MOTIONPLANNING_FORWARDKINEMATICS_H_

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace motion_planning {

class ForwardKinematics {
public:
	ForwardKinematics();
	virtual ~ForwardKinematics();

	/**
	 * Function to compute Forward kinematics given the joint angles. basically it would provide you
	 * end-effector location based on joint angles
	 *
	 * @param joint_angles vector of Joint angles
	 * @param trans_mat a 4X4 transformation matrix returning end effector position and orientation
	 *
	 * return 0 if successful
	 */
	int computeFK(std::vector<double> joint_angles, Eigen::MatrixXd& trans_mat);

private:
	std::vector<double> alpha, a, d, theta;
};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_FORWARDKINEMATICS_H_ */
