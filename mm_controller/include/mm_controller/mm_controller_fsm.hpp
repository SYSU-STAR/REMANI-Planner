#ifndef __CONTROL_FSM_H
#define __CONTROL_FSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "mm_controller/controller_utils.hpp"
#include "mm_controller/polynomial_trajectory.h"

namespace MMController{


class MMControllerFSM
{
public:

	State_Data_t state_data;
	Command_Data_t cmd_data;
	Trajectory_Data_t trajectory_data;

	Eigen::VectorXd stay_joint_;
	Eigen::Vector2d stay_pos_;
	double stay_yaw_;

	enum State_t{
		AUTO_STAY,
		CMD_CTRL,
	};

    enum Exec_Traj_State_t{
		STAY = 10, // execute the hover trajectory 
		POLY_TRAJ = 11, // execute the polynomial trajectory
	};

	State_t last_state_;

	MMControllerFSM(const ros::NodeHandle& nh);

	void process();
	void CMD_CTRL_process();
	bool state_is_received(const ros::Time &now_time);

private:
	int mobile_base_dof_, manipulator_dof_;
	ros::Publisher traj_start_trigger_pub_;
	ros::Publisher car_cmd_pub_, mani_cmd_pub_, gripper_cmd_pub_;
	State_t fsm_state_; // Should only be changed in PX4CtrlFSM::process() function!
	Exec_Traj_State_t exec_traj_state_;
	Eigen::VectorXd init_theta_;
	bool init_gripper_close_;
	int last_singul_;

	// Handles
	ros::NodeHandle nh_;

	void publish_ctrl_cmd(const ros::Time &stamp);
	void update_stay_state(bool mani_goto_init);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
	void cal_traj_ctrl_input(double duration);
	void cal_stay_ctrl_input();
	void limitErr(Eigen::VectorXd &x, const double x_low, const double x_upp);
};

}

#endif