/*
 * Jacobian.h
 *
 *  Created on: 24-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_JACOBIAN_H_
#define INCLUDE_JACOBIAN_H_

#include "ForwardKinematics.h"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace motion_planning {

class Jacobian: public ForwardKinematics {
public:
	Jacobian();
	virtual ~Jacobian();
	int computeJacobian(std::vector<double> joint_angles, Eigen::MatrixXd& jacob_mat);
};

} /* namespace motion_planning */

#endif /* INCLUDE_JACOBIAN_H_ */
