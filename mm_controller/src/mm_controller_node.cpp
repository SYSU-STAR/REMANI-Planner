#include <ros/ros.h>
#include "mm_controller/mm_controller_fsm.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "MMctrl");
    ros::NodeHandle nh("~");
    ros::Duration(1.0).sleep();
    MMController::MMControllerFSM fsm(nh);

    ros::Subscriber car_odom_sub = 
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&MMController::State_Data_t::feed_odom, &fsm.state_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber joint_state_sub =
        nh.subscribe<sensor_msgs::JointState>("joint_states",
                                         100,
                                         boost::bind(&MMController::State_Data_t::feed_joint, &fsm.state_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber traj_sub =
        nh.subscribe<quadrotor_msgs::PolynomialTraj>("planning/trajectory",
                                                      100,
                                                      boost::bind(&MMController::Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Duration(0.5).sleep();

    ros::Rate r(100);
    while (ros::ok()){
        r.sleep();
        ros::spinOnce();
        fsm.process();  // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}

