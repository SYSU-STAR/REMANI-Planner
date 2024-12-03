#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <mm_config/mm_config.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

int _manipulator_dof;

ros::Subscriber car_cmd_sub;
ros::Subscriber joint_cmd_sub, gripper_cmd_sub;
ros::Publisher  car_odom_pub;
ros::Publisher  joint_state_pub, gripper_state_pub, gripper_angle_pub;

double _init_x, _init_y, _init_yaw;
Eigen::VectorXd _init_theta;

bool rcv_car_cmd   = false;
bool rcv_joint_cmd = false;
bool gripper_state_ = false; // false: open true: close
double gripper_angle_ = M_PI / 6; // M_PI / 6: open 0: close
bool rcv_gripper_state_ = false;
geometry_msgs::Twist _car_vel_cmd;
control_msgs::JointTrajectoryControllerState _joint_pos_cmd;
nav_msgs::Odometry _last_odom;
sensor_msgs::JointState _last_joint_state;
double _last_yaw;

double _time_resolution = 0.001;

ros::Time last_cmd_time_ = ros::Time(0);

void rcvCarVelCmdCallBack(const geometry_msgs::Twist cmd){	
	rcv_car_cmd  = true;
	_car_vel_cmd = cmd;
	last_cmd_time_ = ros::Time::now();
}

void rcvJointCmdCallBack(const control_msgs::JointTrajectoryControllerState cmd){ // pos cmd
	rcv_joint_cmd = true;
	_joint_pos_cmd = cmd;
	last_cmd_time_ = ros::Time::now();
}

void rcvGripperCmdCallBack(const std_msgs::Bool cmd){
	if(!rcv_gripper_state_){
		rcv_gripper_state_ = true;
		if(cmd.data){
			gripper_angle_ = 0;
		}else{
			gripper_angle_ = M_PI / 6;
		}
	}
	gripper_state_ = cmd.data;
}

void normyaw(double& y){
	if (y >= M_PI){
		y -= 2 * M_PI;
	}
	else if (y < -M_PI){
		y += 2 * M_PI;
	}
}

void pubCarOdom(){
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";
	double yaw;

	if(rcv_car_cmd){
		// true val
		odom.pose.pose.position.x = _car_vel_cmd.linear.x;
		odom.pose.pose.position.y = _car_vel_cmd.linear.y;
		odom.pose.pose.position.z = 0.0;
		yaw = _car_vel_cmd.linear.z;
		normyaw(yaw);
		odom.twist.twist.linear.x = _car_vel_cmd.angular.x;
		odom.twist.twist.linear.y = _car_vel_cmd.angular.y;
		odom.twist.twist.linear.z = 0.0;

		odom.pose.pose.orientation.w = cos(yaw / 2);
		odom.pose.pose.orientation.x = 0.0;
		odom.pose.pose.orientation.y = 0.0;
		odom.pose.pose.orientation.z = sin(yaw / 2);

		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = 0.0;
	}else{
		odom.pose.pose.position.x = _init_x;
	    odom.pose.pose.position.y = _init_y;
	    odom.pose.pose.position.z = 0.0;

	    odom.pose.pose.orientation.w = cos(_init_yaw / 2);
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = sin(_init_yaw / 2);

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;

		yaw = _init_yaw;
	}

    car_odom_pub.publish(odom);
	_last_odom = odom;
	_last_yaw = yaw;
}

void pubJointState(){ // pos cmd
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.header.frame_id = "world";
	std::vector<double> joint_p, joint_v, joint_effort;
	joint_p.clear();
	joint_v.clear();
	joint_effort.clear();
	if(rcv_joint_cmd){
		for(int i = 0; i < _manipulator_dof; ++i){
			joint_effort.push_back(_joint_pos_cmd.desired.effort[i]);
			joint_v.push_back(_joint_pos_cmd.desired.velocities[i]);
			joint_p.push_back(_joint_pos_cmd.desired.positions[i]);
		}
	}else{
		for(int i = 0; i < _manipulator_dof; ++i){
			joint_p.push_back(_init_theta(i));
			joint_v.push_back(0.0);
			joint_effort.push_back(0.0);
		}
	}
	joint_state.position = joint_p;
	joint_state.velocity = joint_v;
	joint_state.effort = joint_effort;
	_last_joint_state = joint_state;
	joint_state_pub.publish(joint_state);

	std_msgs::Bool gripper_state;
	gripper_state.data = gripper_state_;
	gripper_state_pub.publish(gripper_state);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

	nh.param("mm/manipulator_dof", _manipulator_dof, -1);
	
	std::vector<double> init_state;
    nh.getParam("fsm/init_state", init_state);
	_init_x = init_state[0];
	_init_y = init_state[1];
	nh.param("fsm/init_yaw", _init_yaw,  0.0);
	_init_yaw = _init_yaw / 180.0 * M_PI;
	_init_theta = Eigen::VectorXd::Zero(_manipulator_dof);
	for(int i = 0; i < _manipulator_dof; i++){
    	_init_theta(i) = init_state[i + 2] * M_PI / 180.0;
    }
	nh.param("fsm/waypoint0_gripper_close", gripper_state_,  false);

	std::vector<double> joint_p, joint_v, joint_effort;
	joint_p.clear();
	joint_v.clear();
	joint_effort.clear();
	for(int i = 0; i < _manipulator_dof; ++i){
		joint_p.push_back(_init_theta(i));
		joint_v.push_back(0.0);
		joint_effort.push_back(0.0);
	}
	_last_joint_state.position = joint_p;
	_last_joint_state.velocity = joint_v;
	_last_joint_state.effort = joint_effort;

	car_cmd_sub     = nh.subscribe("/mm_controller_node/car_cmd", 1, rcvCarVelCmdCallBack);
	joint_cmd_sub   = nh.subscribe("/mm_controller_node/joint_cmd", 1, rcvJointCmdCallBack);
	gripper_cmd_sub   = nh.subscribe("gripper_cmd", 1, rcvGripperCmdCallBack);
	car_odom_pub    = nh.advertise<nav_msgs::Odometry>("odometry", 1);
	joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
	gripper_state_pub = nh.advertise<std_msgs::Bool>("gripper_state", 1);
	gripper_angle_pub = nh.advertise<std_msgs::Float64>("gripper_angle", 1);

    ros::Rate rate(1.0 / _time_resolution);
    bool status = ros::ok();
    while(status){
		pubCarOdom();
		pubJointState();
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}