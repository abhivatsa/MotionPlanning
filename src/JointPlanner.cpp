/*
 * JointPlanner.cpp
 *
 *  Created on: 24-May-2023
 *      Author: shubham
 */

#include <MotionPlanning/JointPlanner.h>

namespace motion_planning {

JointPlanner::JointPlanner() {
	// TODO Auto-generated constructor stub
	MAX_JOINT_VEL.resize(numJoints, M_PI);
	MAX_JOINT_ACC.resize(numJoints, M_PI);

	total_time = 0;
	this->publish_data = std::bind(&JointPlanner::defaultPublishFunction, this, std::placeholders::_1);
//	this->_watchdog = std::bind(&JointPlanner::defaultWatchdogFunction, this);
}

JointPlanner::~JointPlanner() {
	// TODO Auto-generated destructor stub
}

int JointPlanner::computePath(std::vector<std::vector<double>> target_jpos, std::vector<std::vector<double>> target_jvel, std::vector<std::vector<double>> target_jacc, std::vector<double> segment_time_, int num_loop) {

	int waypoint_num = target_jpos.size();

	std::cout<<"WayPoints : "<<waypoint_num<<"\n";

	std::vector<double> init_point_pos;
	init_point_pos.resize(numJoints,0); //TODO :  Change it to the current position

	std::cout<<"Computing segment time \n";
	for(size_t i = 0 ; i < waypoint_num; i++)
	{
		if(segment_time_[i] < 0)
		{
			if(i < waypoint_num -1)
				segment_time_[i] = ja_jv_to_time(target_jvel[i], target_jacc[i], target_jpos[i], target_jpos[i+1]);
			else
				segment_time_[i] = ja_jv_to_time(target_jvel[i], target_jacc[i], target_jpos[i], target_jpos[0]);
		}
	}
	std::cout<<"Computing segment time : Done \n";

	total_time = 0;

	std::cout<<"Moving to First Point \n";
	double t = ja_jv_to_time(MAX_JOINT_VEL, MAX_JOINT_ACC, init_point_pos, target_jpos[0]);
	std::vector<TrajectoryPoint> trajectory_points;
	std::vector<double> last_jpos;
	last_jpos = init_point_pos;
	pointToPointFtTime(init_point_pos, target_jpos[0], t, MAX_JOINT_ACC, last_jpos, trajectory_points);

	std::cout<<"Moving to First Point : Done \n";

	std::vector<double> init_jpos, desired_acc, final_jpos;
	double segment_time;

	init_jpos = last_jpos;

	bool not_completed = true;
	int loop_ctr = 1;
	int nxt;
	int curr;

	std::cout<<"Starting Motion Plan \n";
	while(not_completed)
	{
		for (int i = 0; i < waypoint_num; i++)
		{
			nxt = (i+1)%waypoint_num;
			curr = i%waypoint_num;
			init_jpos = target_jpos[curr];
			final_jpos = target_jpos[nxt];
			desired_acc = target_jacc[nxt];
			segment_time = segment_time_[nxt];

			pointToPointFtTime(init_jpos, final_jpos, segment_time, desired_acc, last_jpos, trajectory_points);

		}

		loop_ctr++;
		if(loop_ctr >= num_loop)
		{
			not_completed = false;
		}
		else if(num_loop < 0)
		{
			loop_ctr = 1;
		}
	}
	std::cout<<"Motion Plan : Done \n";

	return 1;
}

double JointPlanner::ja_jv_to_time(std::vector<double> desired_joint_vel, std::vector<double> &desired_acc, std::vector<double> init_jpos, std::vector<double> final_jpos)

{
	//sign computation for vel and acc
	for(unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
	{
		if (final_jpos[joint_ctr] < init_jpos[joint_ctr])
		{
			desired_joint_vel[joint_ctr] = -fabs(desired_joint_vel[joint_ctr]);
			desired_acc[joint_ctr] = -fabs(desired_acc[joint_ctr]);
		}
		else
		{
			desired_joint_vel[joint_ctr] = fabs(desired_joint_vel[joint_ctr]);
			desired_acc[joint_ctr] = fabs(desired_acc[joint_ctr]);
		}

	}


	double slowest_time = 0;

	//iterating over all joints for computing slowest segment time
	for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
	{

		// if desired vel is greater than maximum joint vel limit
		if (fabs(desired_joint_vel[joint_ctr]) > MAX_JOINT_VEL[joint_ctr])
		{
			desired_joint_vel[joint_ctr] = desired_joint_vel[joint_ctr]/fabs(desired_joint_vel[joint_ctr])*MAX_JOINT_VEL[joint_ctr];
		}

		// if desired acc is greater than maximum joint acc limit
		if (fabs(desired_acc[joint_ctr]) > MAX_JOINT_ACC[joint_ctr])
		{
			desired_acc[joint_ctr] = desired_acc[joint_ctr]/fabs(desired_acc[joint_ctr])*MAX_JOINT_ACC[joint_ctr];
		}

		// if initial joint val and final joint val are different
		if (init_jpos[joint_ctr] != final_jpos[joint_ctr])
		{

			double theta_diff = final_jpos[joint_ctr] - init_jpos[joint_ctr];
			double max_vel_possible = sqrt(2*desired_acc[joint_ctr]*theta_diff/2);//derived from newton's equation of motion based on const. acc {v^2 = 2*a*(S/2)}

			// if max_joint vel is greater than max joint vel possible in between 2 points
			if (fabs(desired_joint_vel[joint_ctr]) > max_vel_possible)
			{
				desired_joint_vel[joint_ctr] = desired_joint_vel[joint_ctr]/fabs(desired_joint_vel[joint_ctr])*max_vel_possible;
			}


			double time_acc, time_cruise, dist_acc, total_time_;

			//computing acc, decc, cruise times assuming trapezoidal velocity profile
			time_acc = desired_joint_vel[joint_ctr]/desired_acc[joint_ctr];

			dist_acc =  desired_joint_vel[joint_ctr]*time_acc/2;

			time_cruise = (theta_diff - 2*dist_acc)/desired_joint_vel[joint_ctr];


			if(fabs(time_cruise) < 1e-5){
				time_cruise = 0;
			}


			total_time_ = time_acc + time_cruise + time_acc;

			//updating global slowest time
			if (total_time_ >= slowest_time)
			{
				slowest_time = total_time_;
			}

		}
		else //if both joint positions are same
		{
			desired_joint_vel[joint_ctr] = 0.0;
			desired_acc[joint_ctr] = 0.0;
		}

	}

	for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
	{
		//checking validity of slowest time based on joint acc and correcting it. Ref. "Introduction to Robotics. Mechanics and Control by John J Craig"
		if ( 4*fabs(final_jpos[joint_ctr] - init_jpos[joint_ctr])/(slowest_time*slowest_time) > fabs(desired_acc[joint_ctr]))
		{
			slowest_time = sqrt(4*fabs(final_jpos[joint_ctr] - init_jpos[joint_ctr])/fabs(0.9*desired_acc[joint_ctr]));
		}
	}

	return slowest_time;
}

int JointPlanner::pointToPointFtTime(std::vector<double> init_jpos,
		std::vector<double> final_jpos, double &segment_time,
		std::vector<double> desired_acc, std::vector<double> &last_jpos, std::vector<TrajectoryPoint> &trajectory_points)
{
	trajectory_points.clear();
	for(unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
	{
		if (final_jpos[joint_ctr] < init_jpos[joint_ctr])
		{
			desired_acc[joint_ctr] = -fabs(desired_acc[joint_ctr]);
		}
		else
		{
			desired_acc[joint_ctr] = fabs(desired_acc[joint_ctr]);
		}

		if(fabs(desired_acc[joint_ctr]) > MAX_JOINT_ACC[joint_ctr])
		{
			desired_acc[joint_ctr] = desired_acc[joint_ctr]/fabs(desired_acc[joint_ctr]) * MAX_JOINT_ACC[joint_ctr];
		}
	}

	if (segment_time < 1e-4)
		segment_time = 0;

	double slowest_acc_period = 0;

	std::vector<double> acc_period;
	acc_period.resize(numJoints, 0.0);


	bool planning_done = false;
	bool increase_curr_seg_time = false;


	while(planning_done == false)
	{
		planning_done = true;
		if(increase_curr_seg_time == true)
		{
			segment_time = segment_time * 1.1;
		}
		increase_curr_seg_time = false;

		for (size_t joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
		{
			double square = desired_acc[joint_ctr]*desired_acc[joint_ctr]*segment_time*segment_time- 4*desired_acc[joint_ctr]*(final_jpos[joint_ctr] - init_jpos[joint_ctr]);

			if(square < 0 && fabs(square)>1e-4)
			{
				segment_time = 1.1 * sqrt(4*fabs((final_jpos[joint_ctr] - init_jpos[joint_ctr])/desired_acc[joint_ctr]));
				planning_done = false;
				break;
			}

			if(fabs(square)<1e-4)
			{
				square=0;
			}

			acc_period[joint_ctr] = segment_time/2 - sqrt(square)/fabs(2*desired_acc[joint_ctr]);

			//storing global slowest time
			if (acc_period[joint_ctr] > slowest_acc_period)
			{
				slowest_acc_period = acc_period[joint_ctr];
			}

		}

		if(planning_done == false)
		{
			continue;
		}
		//Re-computing desired joint acc based on the global slowest time
		for (size_t joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
		{
			if(fabs(final_jpos[joint_ctr] - init_jpos[joint_ctr]) < 1e-4)
			{
				desired_acc[joint_ctr] = 0;
			}
			else
			{
				desired_acc[joint_ctr] = (final_jpos[joint_ctr] - init_jpos[joint_ctr])/(segment_time*slowest_acc_period -slowest_acc_period*slowest_acc_period);
			}
			acc_period[joint_ctr] = slowest_acc_period;
		}

		for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
		{
			if(fabs(desired_acc[joint_ctr]*slowest_acc_period) > MAX_JOINT_VEL[joint_ctr])
			{
				planning_done = false;
				increase_curr_seg_time = true;
				break;
			}
		}
	}

	bool not_completed = true;
	double time = 0;

	//Loop for trajextory generation and execution

	TrajectoryPoint start_point;
	start_point.positions = init_jpos;
	start_point.velocities.resize(init_jpos.size(), 0);
	start_point.accelerations.resize(init_jpos.size(), 0);
	start_point.time_from_start = 0;
	start_point.total_time = total_time;

	trajectory_points.push_back(start_point);

	while(not_completed)
	{
		time = time + 0.025;//total trajectory time

		std::vector<double> joint_position_curr, joint_velocity_curr, joint_acceleration_curr;
		joint_position_curr.resize(numJoints);
		joint_velocity_curr.resize(numJoints);
		joint_acceleration_curr.resize(numJoints);

		//Determining the trajecory phase over all joints
		for (unsigned int joint_ctr = 0; joint_ctr < numJoints; joint_ctr++)
		{

			if (time < acc_period[joint_ctr])//Trajectory generation for acc phase
			{
				//Derived from Newton's equations of motion
				joint_position_curr[joint_ctr] = init_jpos[joint_ctr] + 0.5*desired_acc[joint_ctr]*time*time;
				joint_velocity_curr[joint_ctr] = desired_acc[joint_ctr]*time;
				joint_acceleration_curr[joint_ctr] = desired_acc[joint_ctr];
			}
			else if(time < (segment_time-acc_period[joint_ctr]))//Trajectory generation for cruise phase
			{
				//Derived from Newton's equations of motion
				joint_position_curr[joint_ctr] = init_jpos[joint_ctr] + 0.5*desired_acc[joint_ctr]*acc_period[joint_ctr]*acc_period[joint_ctr] + desired_acc[joint_ctr]*acc_period[joint_ctr]*(time - acc_period[joint_ctr]);
				joint_velocity_curr[joint_ctr] = desired_acc[joint_ctr]*acc_period[joint_ctr];
				joint_acceleration_curr[joint_ctr] = 0;
			}
			else if (time < (segment_time))//Trajectory generation for decc phase
			{
				//Derived from Newton's equations of motion
				joint_position_curr[joint_ctr] = init_jpos[joint_ctr] + 0.5*desired_acc[joint_ctr]*acc_period[joint_ctr]*acc_period[joint_ctr] + desired_acc[joint_ctr]*acc_period[joint_ctr]*(segment_time-2*acc_period[joint_ctr]) + desired_acc[joint_ctr]*acc_period[joint_ctr]*(time - (segment_time-acc_period[joint_ctr])) - 0.5*desired_acc[joint_ctr]*(time - (segment_time-acc_period[joint_ctr]))*(time - (segment_time-acc_period[joint_ctr]));
				joint_velocity_curr[joint_ctr] = desired_acc[joint_ctr]*acc_period[joint_ctr] - desired_acc[joint_ctr]*(time - (segment_time-acc_period[joint_ctr]));
				joint_acceleration_curr[joint_ctr] = -desired_acc[joint_ctr];
			}
			else //Trajectory generation for trajectory completion
			{
				//Setting directly to target
				joint_position_curr[joint_ctr] = final_jpos[joint_ctr];
				joint_velocity_curr[joint_ctr] = 0.0;
				joint_acceleration_curr[joint_ctr] = -desired_acc[joint_ctr];
				not_completed = false;
			}
		}


		if (fabs(time - segment_time) < 1e-5 || time > segment_time)//Traj time has ended. Set final values directly
		{
			for(unsigned int joint_ctr = 0; joint_ctr< numJoints; joint_ctr++)
			{
				joint_position_curr[joint_ctr] = final_jpos[joint_ctr];
				joint_velocity_curr[joint_ctr] = 0.0;
				joint_acceleration_curr[joint_ctr] = 0.0;
			}

			not_completed = false;
		}


		if (not_completed == false) //Traj completed. Set target valuues and times directly
		{
			last_jpos = joint_position_curr;
		}

		TrajectoryPoint point;
		point.positions = joint_position_curr;
		point.velocities = joint_velocity_curr;
		point.accelerations = joint_acceleration_curr;
		point.time_from_start = time;
		point.total_time = total_time + time;

		trajectory_points.push_back(point);

	}

	publish_data(trajectory_points);

	total_time = total_time + time;
	return 1;

}

void JointPlanner::AddPublisher(
		std::function<void(std::vector<TrajectoryPoint>)> publish_function) {
	this->publish_data = publish_function;
}

//void JointPlanner::AddWatchdog(std::function<void(void)> watchdog_function) {
//	this->_watchdog = watchdog_function;
//}

void JointPlanner::defaultPublishFunction(
		std::vector<TrajectoryPoint> traj_points) {

}

void JointPlanner::setJointVelAccLimit(std::vector<double> joint_vel_limit,
		std::vector<double> joint_acc_limit) {
	if(joint_vel_limit.size() != numJoints || joint_acc_limit.size() != numJoints)
	{
		std::cout<<"Invalid vector size, Using Default values\n";
		return;
	}
	for(int i = 0; i< numJoints; i++)
	{
		MAX_JOINT_VEL[i] = fabs(joint_vel_limit[i]);
		MAX_JOINT_ACC[i] = fabs(joint_acc_limit[i]);
	}
}

//void JointPlanner::defaultWatchdogFunction() {
//}

} /* namespace motion_planning */
