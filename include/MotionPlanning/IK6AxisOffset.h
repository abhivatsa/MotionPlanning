/*
 * IK6AxisOffset.h
 *
 *  Created on: 22-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_MOTIONPLANNING_IK6AXISOFFSET_H_
#define INCLUDE_MOTIONPLANNING_IK6AXISOFFSET_H_

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace motion_planning {

class IK6AxisOffset {
public:
	IK6AxisOffset();
	virtual ~IK6AxisOffset();
	int computeIK(Eigen::Vector3d eef_pos, Eigen::Matrix3d eef_orient,
	    		std::vector<double>& current_joint_val, std::vector<double>& joint_val);

private:
	double d1, d4, d5, d6, a2, a3;

};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_IK6AXISOFFSET_H_ */
