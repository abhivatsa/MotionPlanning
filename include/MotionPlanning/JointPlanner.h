/*
 * JointPlanner.h
 *
 *  Created on: 24-May-2023
 *      Author: shubham
 */

#ifndef INCLUDE_MOTIONPLANNING_JOINTPLANNER_H_
#define INCLUDE_MOTIONPLANNING_JOINTPLANNER_H_

#include <iostream>
#include <vector>
#include <functional>

namespace motion_planning {

struct TrajectoryPoint {
	std::vector<double> positions;
	std::vector<double> velocities;
	std::vector<double> accelerations;
	double time_from_start;
	double total_time;
};

class JointPlanner {
public:
	JointPlanner();
	virtual ~JointPlanner();

	void setJointVelAccLimit(std::vector<double> joint_vel_limit, std::vector<double> joint_acc_limit);

	int computePath(std::vector<std::vector<double>> target_jpos, std::vector<std::vector<double>> target_jvel, std::vector<std::vector<double>> target_jacc, std::vector<double> segment_time, int num_loop);

	/** Add a custom publisher function (for ros1 and ros2). This function will be called at the end of each segment.
	 * \param[in] callback_function of type void (std::vector<motion_planning::TrajectoryPoint>) (e.g., auto callback_function = std::bind(${NAME_OF_FUNCTION},std::placeholders::_1);)
	 */
	void AddPublisher(std::function<void(std::vector<TrajectoryPoint>)> publish_function);

protected:
	/** Compute segment time from joint positions, velcities and accelerations
	 *
	 */
	double ja_jv_to_time(std::vector<double> desired_joint_vel, std::vector<double> &desired_joint_acc, std::vector<double> init_joint_val, std::vector<double> final_joint_val);

	/** Computes the trajectory points for point to point motion in joint space with zero final and initial velocities. At the end of the computation, it calls user defined callback function (via addPublisher)
	 *
	 */
	int pointToPointFtTime(std::vector<double> init_jpos, std::vector<double> final_jpos, double &segment_time, std::vector<double> desired_acc, std::vector<double> &last_jpos, std::vector<TrajectoryPoint> &trajectory_points);
	std::function<void(std::vector<TrajectoryPoint> traj_points)> publish_data;

	int numJoints = 6;
	double total_time;

private:
	void defaultPublishFunction(std::vector<TrajectoryPoint> traj_points);
	std::vector<double> MAX_JOINT_VEL;
	std::vector<double> MAX_JOINT_ACC;

};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_JOINTPLANNER_H_ */
