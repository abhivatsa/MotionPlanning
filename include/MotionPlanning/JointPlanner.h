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

	/** Add a custom watchdog function
		 * \param[in] callback_function of type void (void) (e.g., auto callback_function = std::bind(${NAME_OF_FUNCTION});)
		 */
//	void AddWatchdog(std::function<void(void)> watchdog_function);

private:

	double ja_jv_to_time(std::vector<double> desired_joint_vel, std::vector<double> &desired_joint_acc, std::vector<double> init_joint_val, std::vector<double> final_joint_val);

	int pointToPointFtTime(std::vector<double> init_jpos, std::vector<double> final_jpos, double &segment_time, std::vector<double> desired_acc, std::vector<double> &last_jpos, std::vector<TrajectoryPoint> &trajectory_points);

	void defaultPublishFunction(std::vector<TrajectoryPoint> traj_points);
//	void defaultWatchdogFunction();

	std::function<void(std::vector<TrajectoryPoint> traj_points)> publish_data;
//	std::function<void(void)> _watchdog;

	std::vector<double> MAX_JOINT_VEL;
	std::vector<double> MAX_JOINT_ACC;

	int numJoints = 6;

	double total_time;

};

} /* namespace motion_planning */

#endif /* INCLUDE_MOTIONPLANNING_JOINTPLANNER_H_ */
