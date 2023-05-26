/*
 * CartestianPlanner.cpp
 *
 *  Created on: 25-May-2023
 *      Author: shubham
 */

#include <MotionPlanning/CartestianPlanner.h>

namespace motion_planning {

CartestianPlanner::CartestianPlanner() {
	// TODO Auto-generated constructor stub
	MAX_VEL = 1.2; //m/s
	MAX_ACC = 1.2; //m/s^2

}

CartestianPlanner::~CartestianPlanner() {
	// TODO Auto-generated destructor stub
}


int CartestianPlanner::computePath(std::vector<std::vector<double>> target_pos_, std::vector<double> target_vel, std::vector<double> target_acc, std::vector<double> target_time, std::vector<bool> orient_const, int num_loops)
{

	int waypoint_num = target_pos_.size();


	Eigen::Isometry3d next_pos, curr_pos, prev_pos;

	std::vector<Eigen::Isometry3d> target_pos;

	for(int i = 0; i < waypoint_num; i++)
	{
		Eigen::Vector3d pos;
		Eigen::Quaterniond rot;

		pos.x() = target_pos_[i][0];
		pos.y() = target_pos_[i][1];
		pos.z() = target_pos_[i][2];

		rot.x() = target_pos_[i][3];
		rot.y() = target_pos_[i][4];
		rot.z() = target_pos_[i][5];
		rot.w() = target_pos_[i][6];

		Eigen::Isometry3d temp_pos;
		temp_pos = Eigen::Isometry3d(rot);
		temp_pos.translation() = pos;

		target_pos.push_back(temp_pos);

		if(target_vel[i] > MAX_VEL)
			target_vel[i] = MAX_VEL;
		else if(target_vel[i] < 0.0001)
			target_vel[i] = 0.0001;

		if(target_acc[i] > MAX_ACC)
			target_acc[i] = MAX_ACC;
		else if(target_acc[i] < 0.0001)
			target_acc[i] = 0.0001;

		if(target_time[i] < 0.0001)
			target_time[i] = 0.001;

		//		std::cout<<target_pos[i].translation()<<"\n\n";
	}

	//	return 1;
	for(int i = 0; i < waypoint_num; i++)
	{
		if(isnan(target_vel[i]))
		{
			if(i < waypoint_num - 1)
				time_to_jv_ja(target_pos[i], target_pos[i+1], target_time[i], target_vel[i], target_acc[i]);
			else
				time_to_jv_ja(target_pos[i], target_pos[0], target_time[i], target_vel[i], target_acc[i]);
		}

	}


	std::vector<double> current_jpos, desired_jvel, desired_jacc, final_jpos, last_jpos;
	current_jpos.resize(6,0);
	desired_jvel.resize(6,M_PI/2);
	desired_jacc.resize(6,M_PI/2);

	last_jpos = current_jpos;

	Eigen::Vector3d pos_=target_pos[0].translation();
	Eigen::Quaterniond rot_(target_pos[0].rotation());
	Eigen::Matrix3d eef_orient = rot_.matrix();


	std::cout<<"Move to 1st point in J-space : Start\n";

	std::vector<TrajectoryPoint> trajectory_points;

	ik_solver.computeIK(pos_, eef_orient, current_jpos, final_jpos);

	//	Eigen::MatrixXd trans_mat1;
	//	fk_solver.computeFK(final_jpos, trans_mat1);
	//
	//	std::cout<<trans_mat1<<"\n";
	//	return 1;

	double start_time = JointPlanner::ja_jv_to_time(desired_jvel, desired_jacc, current_jpos, final_jpos);
	JointPlanner::pointToPointFtTime(current_jpos, final_jpos, start_time, desired_jacc, last_jpos, trajectory_points);

	std::cout<<"Move to 1st point in J-space : Done\n";

	Eigen::MatrixXd trans_mat;
	fk_solver.computeFK(last_jpos, trans_mat);

	Eigen::Vector3d temp_pos(trans_mat.col(3)[0],trans_mat.col(3)[1],trans_mat.col(3)[2]);
	Eigen::Matrix3d temp_orient = trans_mat.topLeftCorner(3, 3);

	curr_pos = Eigen::Isometry3d(temp_orient);
	curr_pos.translation() = temp_pos;

	prev_pos = curr_pos;


	bool not_completed = true;
	int loop_ctr = 1;
	int nxt;
	int curr;

	double des_acc, segment_time, des_vel;
	bool orient_constraint;
	std::cout<<"Starting Motion Plan \n";
	while(not_completed)
	{
		for (int i = 0; i < waypoint_num; i++)
		{
			nxt = (i+1)%waypoint_num;
			curr = i%waypoint_num;
			next_pos = target_pos[nxt];
			des_acc = target_acc[nxt];
			segment_time = target_time[nxt];
			des_vel = target_vel[nxt];
			orient_constraint =  orient_const[nxt];

			std::cout<<"point to point : started\n";
			pointToPointFtTime(curr_pos, next_pos, des_vel, des_acc, prev_pos, last_jpos, orient_constraint, trajectory_points);
			std::cout<<"point to point : finished\n";

			curr_pos = prev_pos;
		}

		loop_ctr++;
		if(loop_ctr >= num_loops)
		{
			not_completed = false;
		}
		else if(num_loops < 0)
		{
			loop_ctr = 1;
		}
	}
	std::cout<<"Motion Plan : Done \n";

	return 1;
}

int CartestianPlanner::time_to_jv_ja(Eigen::Isometry3d init_pose,
		Eigen::Isometry3d final_pose, double segment_time, double &des_vel,
		double &des_acc) {
	Eigen::Vector3d pt1 = init_pose.translation();
	Eigen::Vector3d pt2 = final_pose.translation();

	double acc = 0;
	double MAX_ACC = 1.2;

	Eigen::Vector3d l1;
	l1 = pt2 - pt1;
	double len_l1 = l1.norm();

	acc = 4*len_l1/(segment_time*segment_time);

	if (acc > MAX_ACC)
	{
		acc = MAX_ACC;
		segment_time = sqrt(4*len_l1/acc);
	}

	double sqrt_=(segment_time*segment_time - (4*len_l1/acc));
	if(fabs(sqrt_)<=1e-5)
	{
		sqrt_=0;
	}
	double acc_time = (segment_time - sqrt(sqrt_))/2;


	double vel = acc*acc_time;

	if (vel > MAX_VEL)
	{
		vel = MAX_VEL;
		acc_time = vel/acc;
		segment_time = acc_time + len_l1/(acc*acc_time);
	}

	sqrt_=segment_time*segment_time - (4*len_l1/acc);
	if(fabs(sqrt_)<=1e-5)
	{
		sqrt_=0;
	}
	acc_time = (segment_time - sqrt(sqrt_))/2;


	vel = acc*acc_time;


	des_vel = fabs(vel);
	des_acc = fabs(acc);

	return 1;
}


int CartestianPlanner::pointToPointFtTime(Eigen::Isometry3d init_pos,
		Eigen::Isometry3d final_pos, double desired_vel, double desired_acc,
		Eigen::Isometry3d &prev_pos, std::vector<double> prev_jpos,
		bool orient_constraint, std::vector<TrajectoryPoint> &trajectory_points) {

//	std::cout<<"point to point : init pos : "<<init_pos.translation()<<"\n";
//	std::cout<<"point to point : "<<"next pos : "<<final_pos.translation()<<"\n";
	//	std::cout<<"point to point : "<<"des vel : "<<desired_vel<<"\n";
	//	std::cout<<"point to point : "<<"des acc : "<<desired_acc<<"\n";
	//	std::cout<<"point to point : "<<"prev pos : "<<prev_pos.translation()<<"\n";
	//	std::cout<<"point to point : "<<"prev jpos : ("<<prev_jpos[0]<<","<<prev_jpos[1]<<","<<prev_jpos[2]<<","<<prev_jpos[3]<<","<<prev_jpos[4]<<","<<prev_jpos[5]<<")\n";

	trajectory_points.clear();
	double vel = 0;
	Eigen::Vector3d pt1 = init_pos.translation();
	Eigen::Vector3d pt2 = final_pos.translation();
	Eigen::Quaterniond initial_quaternion(init_pos.rotation());
	Eigen::Quaterniond final_quaternion(final_pos.rotation());

	std::vector<double> joint_positions;

	joint_positions.resize(numJoints);

	double traj_time = 0;

	if (pt1 == pt2)
	{
		return 1;
	}

	Eigen::Vector3d l1 = pt2 - pt1;
	double len_l1 = l1.norm();                                           /* Amount of distance to travel */

	double initial_vel = 0;
	double final_vel = 0;

	double max_vel = sqrt((initial_vel*initial_vel + final_vel*final_vel + 2*desired_acc*len_l1)/2);

	double acc_period, cruise_period, deacc_period, acc_dist, deacc_dist, cruise_dist;

	if (desired_vel > max_vel)
	{
		vel = max_vel;
	}
	else{
		vel = desired_vel;
	}
	int acc_sign;

	if (initial_vel >= vel)
	{
		acc_sign = -1;
		if (final_vel != 0.0)
		{
			acc_period = 0;
			cruise_period = 0;
			acc_dist = 0;
			cruise_dist = 0;
			deacc_period = (initial_vel - final_vel)/desired_acc;
			deacc_dist = (initial_vel*initial_vel - final_vel*final_vel)/(2*desired_acc);
		}
		else{
			acc_period = (vel - initial_vel)/(desired_acc*acc_sign);
			acc_dist = (vel*vel - initial_vel*initial_vel)/(2*desired_acc*acc_sign);
			deacc_period = (vel - final_vel)/desired_acc;
			deacc_dist = (vel*vel - final_vel*final_vel)/(2*desired_acc);
			cruise_dist = 0;
			cruise_period = 0;
			if (fabs(deacc_dist + acc_dist -len_l1)>1e-5)
			{
				cruise_dist = len_l1 - deacc_dist - acc_dist;
				cruise_period = cruise_dist/vel;
			}
		}
	}
	else{
		acc_sign = 1;
		acc_period = (vel - initial_vel)/desired_acc;
		deacc_period = vel/desired_acc;
		acc_dist = initial_vel*acc_period + 0.5*desired_acc*acc_period*acc_period;
		deacc_dist = vel*vel/(2*desired_acc);
		cruise_dist = len_l1 - acc_dist - deacc_dist;
		cruise_period = cruise_dist/vel;
	}



	if (fabs(cruise_dist) < 1e-5)
	{
		cruise_period = 0;
		cruise_dist = 0;
	}

	double step_acc_dist, step_cruise_dist, step_deacc_dist;

	// Acceeleration step computation
	if(fabs(acc_dist/10)>0.002)
	{
		step_acc_dist=0.002;
	}
	else
	{
		step_acc_dist=fabs(acc_dist)/10;
	}

	// cruise step computation
	if(fabs(cruise_dist/10)>0.002)
	{
		step_cruise_dist=0.002;
	}
	else
	{
		step_cruise_dist=fabs(cruise_dist)/10;
	}

	// Deacceeleration step computation
	if(fabs(deacc_dist/10)>0.002)
	{
		step_deacc_dist=0.002;
	}
	else
	{
		step_deacc_dist=fabs(deacc_dist)/10;
	}


	Eigen::Vector3d l1_unit_vec = l1;
	l1_unit_vec.normalize();

	double percentage_completed(0.0);

	//    Variables for determination of Intermediate waypoints
	Eigen::Vector3d pt_loc;
	Eigen::Quaterniond loc_quat;

	bool not_completed = false;
	double dist = 0;
	double time_val = 0;

	TrajectoryPoint start_point;
	start_point.positions = prev_jpos;
	start_point.velocities.resize(numJoints, 0);
	start_point.accelerations.resize(numJoints, 0);
	start_point.time_from_start = 0.001;
	start_point.total_time = total_time;

	trajectory_points.push_back(start_point);

	while (!not_completed)
	{

		if ((fabs(acc_dist)-fabs(dist) )>1e-6)
		{
			if ((fabs(acc_dist) - fabs(dist)) > (step_acc_dist + 1e-6))
			{
				dist = dist + step_acc_dist;
				pt_loc = pt1 + dist*l1_unit_vec;
				time_val  = acc_sign*(sqrt(initial_vel*initial_vel + 2*acc_sign*desired_acc*step_acc_dist) - initial_vel)/desired_acc;
			}
			else
			{
				time_val  = acc_sign*(vel - initial_vel)/desired_acc;
				dist = acc_dist;
				pt_loc = pt1 + dist*l1_unit_vec;
			}
			initial_vel = initial_vel + acc_sign*desired_acc*time_val;
		}
		else if((fabs(acc_dist+cruise_dist)-fabs(dist))>1e-6)
		{
			if ((fabs(acc_dist+cruise_dist) - fabs(dist)) > (step_cruise_dist + 1e-6))
			{
				dist = dist + step_cruise_dist;
				pt_loc = pt1 + dist*l1_unit_vec;
				time_val  = step_cruise_dist/vel;
			}
			else
			{
				time_val  = (fabs(acc_dist+cruise_dist) - fabs(dist))/vel;
				dist = acc_dist+cruise_dist;
				pt_loc = pt1 + dist*l1_unit_vec;
			}
			initial_vel = vel;
		}
		else if((fabs(acc_dist+cruise_dist+deacc_dist)-fabs(dist))>1e-6)
		{
			if ((fabs(acc_dist+cruise_dist+deacc_dist) - fabs(dist)) > (step_deacc_dist + 1e-6))
			{
				dist = dist + step_deacc_dist;
				pt_loc = pt1 + dist*l1_unit_vec;
				time_val  = (-sqrt(initial_vel*initial_vel - 2*desired_acc*step_deacc_dist) + initial_vel)/desired_acc;

			}
			else
			{
				time_val = (initial_vel - final_vel)/desired_acc;
				dist = acc_dist+cruise_dist+deacc_dist;
				pt_loc = pt2;
				not_completed=true;
			}
			initial_vel = initial_vel - desired_acc*time_val;
		}

		if(fabs(fabs(dist)-fabs(len_l1))<1e-5)
		{
			not_completed=true;
		}

		if(std::isnan(time_val) || time_val<1e-6)
		{
			time_val=0.001;
		}

		traj_time = traj_time + time_val;

		//      Evaluating the pose at intermediate waypoints
		percentage_completed = dist/len_l1;
		Eigen::Isometry3d local_pose;

		if(!orient_constraint)
		{
			local_pose=Eigen::Isometry3d(initial_quaternion.slerp(percentage_completed,final_quaternion));
		}
		else
		{
			local_pose=Eigen::Isometry3d(initial_quaternion);
		}

		local_pose.translation() = pt_loc;


		prev_pos = local_pose;

		bool status;


		Eigen::Vector3d pos_=local_pose.translation();
		Eigen::Quaterniond rot_(local_pose.rotation());


		Eigen::Matrix3d eef_orient = rot_.matrix();

		ik_solver.computeIK(pos_, eef_orient, prev_jpos, joint_positions);


		TrajectoryPoint point;
		point.positions = joint_positions;
		point.time_from_start =traj_time;
		point.total_time = total_time + traj_time;
		trajectory_points.push_back(point);


	}

	total_time = total_time + traj_time;


	publish_data(trajectory_points);

	return 1;
}


} /* namespace motion_planning */
