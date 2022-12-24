/*
 * IK6AxisInline.h
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_IK6AXISINLINE_H_
#define INCLUDE_IK6AXISINLINE_H_

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

private:
	double l0, l1, l2, l3;
};

} /* namespace motion_planning */

#endif /* INCLUDE_IK6AXISINLINE_H_ */
