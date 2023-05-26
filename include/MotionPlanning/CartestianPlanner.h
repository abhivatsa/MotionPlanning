/*
 * CartestianPlanner.h
 *
 *  Created on: 25-May-2023
 *      Author: shubham
 */

#ifndef INCLUDE_MOTIONPLANNING_CARTESTIANPLANNER_H_
#define INCLUDE_MOTIONPLANNING_CARTESTIANPLANNER_H_

#include <iostream>
#include <vector>
#include <functional>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <MotionPlanning/IK6AxisInline.h>
#include <MotionPlanning/ForwardKinematics.h>
#include <MotionPlanning/JointPlanner.h>

namespace motion_planning {


class CartestianPlanner : public JointPlanner {
public:
	CartestianPlanner();
	virtual ~CartestianPlanner();

	int computePath(std::vector<std::vector<double>> target_pos, std::vector<double> target_vel, std::vector<double> target_acc, std::vector<double> target_time, std::vector<bool> orient_const, int num_loops);

private:

	int time_to_jv_ja(Eigen::Isometry3d init_pose, Eigen::Isometry3d final_pose, double segment_time, double& des_vel, double& des_acc);

	int pointToPointFtTime(Eigen::Isometry3d init_pos, Eigen::Isometry3d final_pos, double desired_vel, double desired_acc, Eigen::Isometry3d &prev_pos, std::vector<double> prev_jpos, bool orient_constraint, std::vector<TrajectoryPoint> &trajectory_points);


	ForwardKinematics fk_solver;
	IK6AxisInline ik_solver;

	double MAX_VEL;
	double MAX_ACC;

};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_CARTESTIANPLANNER_H_ */
