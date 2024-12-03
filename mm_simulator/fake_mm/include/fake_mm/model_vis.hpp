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
#include <nav_msgs/Path.h>

namespace model_vis{
    struct State_Data_t{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d car_p;
        Eigen::Quaterniond car_q;
        double car_yaw;
        Eigen::Vector3d car_v;
        Eigen::Vector3d car_w;
        Eigen::VectorXd joint_p;
        Eigen::VectorXd joint_v;

        nav_msgs::Odometry odom_msg;
        sensor_msgs::JointState joint_state_msg;
        int mani_dof_;
        ros::Time rcv_stamp;
        bool need_vis;

        State_Data_t(){
            need_vis = false;
        }

        void setParam(int mani_dof){
            mani_dof_ = mani_dof;
            joint_p.resize(mani_dof_);
            joint_v.resize(mani_dof_);
        }

        void feed_odom(const nav_msgs::Odometry::ConstPtr& pMsg){
            odom_msg = *pMsg;
            rcv_stamp = ros::Time::now();
            need_vis = true;

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

            car_w(0) = odom_msg.twist.twist.angular.x;
            car_w(1) = odom_msg.twist.twist.angular.y;
            car_w(2) = odom_msg.twist.twist.angular.z;
        }

        void feed_joint(const sensor_msgs::JointState::ConstPtr& pMsg){
            joint_state_msg = *pMsg;
            rcv_stamp = ros::Time::now();
            need_vis = true;

            for(int i = 0; i < mani_dof_; ++i){
                // FIXME joint_state must be sent in specific order: 
                joint_p(i) = joint_state_msg.position[i];
                joint_v(i) = joint_state_msg.velocity[i];
            }
        }
    };

    class ModelManager{
    public:
        void init(ros::NodeHandle &nh);

    private:
        int manipulator_dof_;
        bool have_car_odom_;
        bool have_mani_odom_;
        bool gripper_state_;
        bool have_gripper_state_;
        std::shared_ptr<remani_planner::MMConfig> mm_config_;
        tf::TransformBroadcaster broadcaster_;
        ros::Timer tf_timer_;
        std::vector<Eigen::Vector2d> his_traj_;
        visualization_msgs::Marker his_traj_sphere_, his_traj_line_strip_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& state);
        void gripperStateCallback(const std_msgs::Bool::ConstPtr& state);
        void tfTimerCallback(const ros::TimerEvent & event);
        void vis_his_traj(Eigen::Vector2d pt);
        ros::Subscriber joint_state_sub_, odom_sub_, gripper_state_sub_;
        ros::Publisher vis_mm_pub_, vis_mm_check_ball_pub_, vis_his_traj_pub_;
        State_Data_t mm_state_;
    };
}


