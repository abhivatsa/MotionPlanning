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
	int getAlpha(std::vector<double>& alpha_vec);
	int getA(std::vector<double>& a_vec);
	int getD(std::vector<double>& d_vec);
	int getTheta(std::vector<double>& theta_vec);
	int computeFK(std::vector<double> joint_angles, Eigen::MatrixXd& trans_mat);

private:
	std::vector<double> alpha, a, d, theta;
};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_FORWARDKINEMATICS_H_ */
