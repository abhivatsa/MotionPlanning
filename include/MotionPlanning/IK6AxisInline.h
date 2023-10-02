/*
 * IK6AxisInline.h
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_MOTIONPLANNING_IK6AXISINLINE_H_
#define INCLUDE_MOTIONPLANNING_IK6AXISINLINE_H_

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace motion_planning {

class IK6AxisInline {
public:
	IK6AxisInline();
	virtual ~IK6AxisInline();
	int computeIK(Eigen::Vector3d eef_pos, Eigen::Matrix3d eef_orient,
			std::vector<double>& current_joint_val, std::vector<double>& joint_val);

	int computeIK(const std::array<double, 3> &eef_pos, const std::array<double, 3> &eef_rpy,
			const std::array<double, 6> &current_joint_position, std::array<double, 6> &joint_val);

	int computeIK(const double eef_pos[3], const double eef_rpy[3],
				const double current_joint_position[6], double joint_val[6]);


	void quaternionToEuler(const Eigen::Quaterniond &quat, double rpy[3]);
	void eulerToQuaternion(const double rpy[3], Eigen::Quaterniond &quat);

private:
	double l0, l1, l2, l3;
};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_IK6AXISINLINE_H_ */
