#ifndef __CONTROL_UTILS_H
#define __CONTROL_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <deque>

#include "traj_utils/poly_traj_utils.hpp"
#include "traj_utils/plan_container.hpp"
// #include "mm_controller/polynomial_trajectory.h"
#include "mm_utils/utils.h"

namespace MMController{

class State_Data_t
{
public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d car_p;
	Eigen::Quaterniond car_q;
	Eigen::Vector3d car_v;
	double car_v_norm;
	Eigen::Vector3d car_w;
	double car_yaw;
	Eigen::VectorXd joint_p;
	Eigen::VectorXd joint_v;

	nav_msgs::Odometry odom_msg;
	sensor_msgs::JointState joint_state_msg;
	int mani_dof_;
	ros::Time rcv_stamp;
	bool have_car_;
	bool have_mani_;

	State_Data_t(){
		rcv_stamp = ros::Time(0);
		car_q.setIdentity();
		have_car_ = false;
		have_mani_ = false;
	}

	void setParam(int mani_dof){
		mani_dof_ = mani_dof;
		joint_p.resize(mani_dof_);
		joint_v.resize(mani_dof_);
	}
	
	void feed_odom(const nav_msgs::Odometry::ConstPtr& pMsg){
		have_car_ = true;
		odom_msg = *pMsg;
		rcv_stamp = ros::Time::now();

		car_p(0) = odom_msg.pose.pose.position.x;
		car_p(1) = odom_msg.pose.pose.position.y;
		car_p(2) = odom_msg.pose.pose.position.z;

		car_q.w() = odom_msg.pose.pose.orientation.w;
		car_q.x() = odom_msg.pose.pose.orientation.x;
		car_q.y() = odom_msg.pose.pose.orientation.y;
		car_q.z() = odom_msg.pose.pose.orientation.z;
		car_yaw = tf::getYaw(pMsg->pose.pose.orientation);

		car_v(0) = odom_msg.twist.twist.linear.x;
		car_v(1) = odom_msg.twist.twist.linear.y;
		car_v(2) = odom_msg.twist.twist.linear.z;
		car_v = car_q.toRotationMatrix() * car_v;
		car_v_norm = car_v.head(2).norm();

		car_w(0) = odom_msg.twist.twist.angular.x;
		car_w(1) = odom_msg.twist.twist.angular.y;
		car_w(2) = odom_msg.twist.twist.angular.z;
	}

	void feed_joint(const sensor_msgs::JointState::ConstPtr& pMsg){
		have_mani_ = true;
		joint_state_msg = *pMsg;
		rcv_stamp = ros::Time::now();

		for(int i = 0; i < mani_dof_; ++i){
			// FIXME joint_state must be sent in specific order: 
			joint_p(i) = joint_state_msg.position[i];
			joint_v(i) = joint_state_msg.velocity[i];
		}
	}
};

struct oneTraj_Data_t{
  public:
  ros::Time traj_start_time;
  ros::Time traj_end_time;
  poly_traj::Trajectory<7> traj;
};

class Trajectory_Data_t{
public:
	double total_traj_start_time;
	double total_traj_end_time;
	ros::Time total_traj_start_time_ros;
	ros::Time total_traj_end_time_ros;
	int exec_traj = 0; // use for aborting the trajectory, 0 means no trajectory is executing  
						// -1 means the trajectory is aborting, 1 means the trajectory is executing 
	// std::deque<oneTraj_Data_t> traj_queue;
	remani_planner::SingulTrajData singul_traj_data;
	
	Trajectory_Data_t(){
		total_traj_start_time_ros = ros::Time(0);
		total_traj_end_time_ros = ros::Time(0);
		exec_traj = 0;
	}

	// void adjust_end_time(){
	// 	if (traj_queue.size() < 2){
	// 		return;
	// 	}
	// 	for (auto it = traj_queue.begin(); it != (traj_queue.end()-1); it++){
	// 		it->traj_end_time = (it+1)->traj_start_time;
	// 	}
	// }

	void feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg);
};

class Command_Data_t{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	double car_vel_cmd; // control
	Eigen::VectorXd car_true_cmd; // true value
	double car_omega_cmd;
	Eigen::VectorXd joint_pos_cmd;
	Eigen::VectorXd joint_vel_cmd;
	Eigen::VectorXd joint_effort_cmd;
	double yaw;

	ros::Time cal_stamp;

	Command_Data_t(){
		cal_stamp = ros::Time(0);
	}
};

}

#endif
