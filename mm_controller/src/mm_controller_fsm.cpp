#include "mm_controller/mm_controller_fsm.hpp"
using namespace std;

namespace MMController{

MMControllerFSM::MMControllerFSM(const ros::NodeHandle& nh) : nh_(nh){
	nh_.param("mm/mobile_base_dof", mobile_base_dof_, -1);
	nh_.param("mm/manipulator_dof", manipulator_dof_, -1);

	std::vector<double> init_state;
    nh.getParam("fsm/init_state", init_state);
	init_theta_.resize(manipulator_dof_);
    for(int i = 0; i < manipulator_dof_; i++){
        init_theta_(i) = init_state[i + mobile_base_dof_] / 180.0 * M_PI;
    }
	nh.getParam("fsm/init_gripper_close", init_gripper_close_);

	last_singul_ = 0;

	car_cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("car_cmd", 10);
	mani_cmd_pub_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("joint_cmd", 10);
	gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("gripper_cmd", 10);
    traj_start_trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

	state_data.setParam(manipulator_dof_);
	fsm_state_ = AUTO_STAY;
	last_state_ = CMD_CTRL;
	exec_traj_state_ = STAY;
	stay_joint_ = Eigen::VectorXd::Zero(manipulator_dof_);
	stay_yaw_ = 0;
	
	cmd_data.joint_pos_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
	cmd_data.joint_vel_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
	cmd_data.joint_effort_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
}

/* 
        Finite State Machine

           system start
               /
              /
             v           
------> AUTO_STAY          
|         ^   |             
|         |   |             
|         |	  |     
|         |   |
|         |   v
-------- CMD_CTRL

*/

void MMControllerFSM::process(){
	// return;
	ros::Time now_time = ros::Time::now();
	if(!state_data.have_car_ || !state_data.have_mani_) return;

	static bool is_first_time = true;
	if(is_first_time){
		is_first_time = false;
		update_stay_state(false);
		std_msgs::Bool gripper_cmd;
		gripper_cmd.data = init_gripper_close_;
		gripper_cmd_pub_.publish(gripper_cmd);
	}
	
	switch (fsm_state_){
		case AUTO_STAY:
		{
			if(last_state_ == CMD_CTRL){
				last_state_ = AUTO_STAY;
				update_stay_state(true);
			}
			cal_stay_ctrl_input();
			fsm_state_ = CMD_CTRL;
			exec_traj_state_ = STAY;
			ROS_INFO("\033[32m[MMctrl] AUTO_STAY(L1) --> CMD_CTRL(L2)\033[32m");
			publish_trigger(state_data.odom_msg);
			ROS_INFO("\033[32m[MMctrl] TRIGGER sent, allow user command.\033[32m");
			break;
		}

		case CMD_CTRL:
		{
			last_state_ = CMD_CTRL;
			CMD_CTRL_process();
			break;
		}
		default:
			break;
	}
	
	if (fsm_state_ == AUTO_STAY || fsm_state_ == CMD_CTRL){
		publish_ctrl_cmd(now_time);
	}
}
/* 
    Finite State Machine

       CMD_CTRL
           /
          /
         v
      STAY 
      ^   |             
      |   |             
      |   |             
      |   |             
      |   |             
      |   v             
    POLY_TRAJ          


*/
void MMControllerFSM::CMD_CTRL_process()
{
	ros::Time now_time = ros::Time::now();
	switch (exec_traj_state_){
		case STAY:
		{
			if(now_time.toSec() >= trajectory_data.total_traj_start_time &&
			 now_time.toSec() <= trajectory_data.total_traj_end_time && 
			 trajectory_data.exec_traj == 1 && (!trajectory_data.singul_traj_data.singul_traj.empty())){
				// same as the below
				update_stay_state(false);
				double traj_time = now_time.toSec() - trajectory_data.total_traj_start_time;

				cal_traj_ctrl_input(traj_time);

				exec_traj_state_ = POLY_TRAJ;
				ROS_INFO("[MMctrl] Receive the trajectory. STAY --> POLY_TRAJ");
			}else{
				cal_stay_ctrl_input();
			}
			break;
		}
		
		
		case POLY_TRAJ:
		{
			if(now_time.toSec() < (trajectory_data.total_traj_start_time) 
			|| now_time.toSec() > trajectory_data.total_traj_end_time 
			|| trajectory_data.exec_traj != 1 || trajectory_data.singul_traj_data.singul_traj.empty()){
				if(trajectory_data.exec_traj != -1){
					// tracking the end point of the trajectory
					// the hover pose is the end point of the trajectory
					auto & traj_info = trajectory_data.singul_traj_data.singul_traj.back().traj;
					int singul = trajectory_data.singul_traj_data.singul_traj.back().singul;
					stay_joint_ = traj_info.getJuncPos(traj_info.getPieceNum()).tail(manipulator_dof_);
					stay_pos_ = traj_info.getJuncPos(traj_info.getPieceNum()).head(mobile_base_dof_);
					Eigen::Vector2d stay_vel_ = traj_info.getJuncVel(traj_info.getPieceNum()).head(mobile_base_dof_);
					stay_yaw_ = atan2(singul * stay_vel_(1), singul * stay_vel_(0));
				}else{
					update_stay_state(false);
				}
				cal_stay_ctrl_input();
				exec_traj_state_ = STAY;
				ROS_INFO("[MMctrl] Stop execute the trajectory. POLY_TRAJ --> STAY");
				trajectory_data.exec_traj = 0;
			}else{
				update_stay_state(false);
				if(now_time.toSec() < (trajectory_data.singul_traj_data.singul_traj.front().start_time)){
					// the start time of first trajectory should be whole trajectory start time
					trajectory_data.total_traj_start_time = trajectory_data.singul_traj_data.singul_traj.front().start_time;
					cal_stay_ctrl_input();
				}else{
					double traj_time = now_time.toSec() - trajectory_data.singul_traj_data.start_time;
					cal_traj_ctrl_input(traj_time);
				}
			}
			break;
		}
		default:
		{
			exec_traj_state_ = STAY;
			ROS_ERROR("[MMctrl] Unknown exec_traj_state_! Jump to hover");
			break;		
		}
	}
}

void MMControllerFSM::update_stay_state(bool mani_goto_init){
	if(mani_goto_init){
		stay_joint_ = init_theta_;
		for(int i = 0; i < manipulator_dof_; ++i) stay_joint_[i] = init_theta_[i];
	}else if(state_data.have_mani_){
		stay_joint_ = state_data.joint_p;
	}
	if(state_data.have_car_){
		stay_pos_ = state_data.car_p.head(2);
		stay_yaw_ = mm_utils::get_yaw_from_quaternion(state_data.car_q);
	}
}

bool MMControllerFSM::state_is_received(const ros::Time &now_time){
	return (now_time - state_data.rcv_stamp).toSec() < 0.1;
}

void MMControllerFSM::publish_ctrl_cmd(const ros::Time &stamp){
	geometry_msgs::Twist car_cmd;
	// true value
	car_cmd.linear.x = cmd_data.car_true_cmd(0); // x
	car_cmd.linear.y = cmd_data.car_true_cmd(1); // y
	car_cmd.linear.z = cmd_data.car_true_cmd(2); // yaw
	car_cmd.angular.x = cmd_data.car_true_cmd(3); //vx
	car_cmd.angular.y = cmd_data.car_true_cmd(4); //vy
	car_cmd.angular.z = 0.0;

	control_msgs::JointTrajectoryControllerState joint_cmd;
	joint_cmd.header.stamp = stamp;
	for(int i = 0; i < manipulator_dof_; ++i){
		joint_cmd.desired.positions.push_back(cmd_data.joint_pos_cmd(i));
		joint_cmd.desired.velocities.push_back(cmd_data.joint_vel_cmd(i));
		joint_cmd.desired.effort.push_back(cmd_data.joint_effort_cmd(i));
	}
	car_cmd_pub_.publish(car_cmd);
	mani_cmd_pub_.publish(joint_cmd);
}

void MMControllerFSM::publish_trigger(const nav_msgs::Odometry &odom_msg){
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	traj_start_trigger_pub_.publish(msg);
}

void MMControllerFSM::cal_traj_ctrl_input(double duration){
	cmd_data.cal_stamp = ros::Time::now();
	duration = std::min(duration, trajectory_data.singul_traj_data.duration);
	last_singul_ = trajectory_data.singul_traj_data.getSingul(duration);
	double temp_duration = duration;
	trajectory_data.singul_traj_data.locatePieceIdx(temp_duration);

	Eigen::VectorXd pos = trajectory_data.singul_traj_data.getPos(duration);
	Eigen::VectorXd vel = trajectory_data.singul_traj_data.getVel(duration);
	Eigen::VectorXd acc = trajectory_data.singul_traj_data.getAcc(duration);

	double yaw = state_data.car_yaw;
	if(last_singul_ == -1){
		yaw += M_PI;
	}
	// true value
	cmd_data.car_true_cmd.resize(5);
	cmd_data.car_true_cmd(0) = pos(0);
	cmd_data.car_true_cmd(1) = pos(1);
	cmd_data.car_true_cmd(2) = atan2(last_singul_ * vel(1), last_singul_ * vel(0));
	cmd_data.car_true_cmd(3) = vel(0);
	cmd_data.car_true_cmd(4) = vel(1);

	cmd_data.joint_pos_cmd = pos.tail(manipulator_dof_);
	cmd_data.joint_vel_cmd = vel.tail(manipulator_dof_);
	cmd_data.joint_effort_cmd = acc.tail(manipulator_dof_);
}

void MMControllerFSM::cal_stay_ctrl_input(){
	cmd_data.cal_stamp = ros::Time::now();
	cmd_data.car_true_cmd.resize(5);
	cmd_data.car_true_cmd.setZero();
	cmd_data.car_true_cmd(0) = stay_pos_(0);
	cmd_data.car_true_cmd(1) = stay_pos_(1);
	cmd_data.car_true_cmd(2) = stay_yaw_;

	cmd_data.joint_pos_cmd = stay_joint_;
	cmd_data.joint_vel_cmd.setZero();
	cmd_data.joint_effort_cmd.setZero();
}
}
