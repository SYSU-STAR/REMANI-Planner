#include "fake_mm/model_vis.hpp"

namespace model_vis{

    void ModelManager::init(ros::NodeHandle &nh){
        mm_config_.reset(new remani_planner::MMConfig);
        mm_config_->setParam(nh);

        have_car_odom_ = false;
        have_mani_odom_ = false;
        /*  param  */
        nh.param("mm/manipulator_dof", manipulator_dof_, -1);
        
        mm_state_.setParam(manipulator_dof_);
        his_traj_.clear();
        
        his_traj_sphere_.header.frame_id = his_traj_line_strip_.header.frame_id = "world";
        his_traj_sphere_.header.stamp = his_traj_line_strip_.header.stamp = ros::Time::now();
        his_traj_sphere_.type = visualization_msgs::Marker::SPHERE_LIST;
        his_traj_line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
        his_traj_sphere_.action = his_traj_line_strip_.action = visualization_msgs::Marker::ADD;
        his_traj_sphere_.id = 0;
        his_traj_line_strip_.id = 1000;

        his_traj_sphere_.pose.orientation.w = his_traj_line_strip_.pose.orientation.w = 1.0;
        his_traj_sphere_.color.r = his_traj_line_strip_.color.r = 1.0;
        his_traj_sphere_.color.g = his_traj_line_strip_.color.g = 0.0;
        his_traj_sphere_.color.b = his_traj_line_strip_.color.b = 1.0;
        his_traj_sphere_.color.a = his_traj_line_strip_.color.a = 1.0;
        his_traj_sphere_.scale.x = 0.1;
        his_traj_sphere_.scale.y = 0.1;
        his_traj_sphere_.scale.z = 0.1;
        his_traj_line_strip_.scale.x = 0.1 / 2;

        have_gripper_state_ = false;

        /* callback */
        vis_mm_pub_            = nh.advertise<visualization_msgs::MarkerArray>("vis_mm", 100, true);
        vis_mm_check_ball_pub_ = nh.advertise<visualization_msgs::Marker>("vis_mm_check_ball", 100, true);
        joint_state_sub_        = nh.subscribe("joint_state", 1, &ModelManager::jointStateCallback, this);
        gripper_state_sub_      = nh.subscribe("gripper_state", 1, &ModelManager::gripperStateCallback, this);
        odom_sub_               = nh.subscribe("odometry", 1, &ModelManager::odomCallback, this);
        vis_his_traj_pub_       = nh.advertise<visualization_msgs::Marker>("mm_his_traj", 100, true);

        tf_timer_ = nh.createTimer(ros::Duration(0.01), &ModelManager::tfTimerCallback, this);

        std::vector<Eigen::Vector3d> car_pts;
        mm_config_->getCarPts(Eigen::Vector3d(0, 0, 0), car_pts, Eigen::Vector3d(0, 0, 0));
    }

    void ModelManager::tfTimerCallback(const ros::TimerEvent & event){ 
        if(!have_car_odom_ || !have_mani_odom_) return;
        ros::Time time = ros::Time::now();
        broadcaster_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(mm_state_.car_q.x(), mm_state_.car_q.y(), mm_state_.car_q.z(), mm_state_.car_q.w()), 
                tf::Vector3(mm_state_.car_p(0), mm_state_.car_p(1), 0.0)),
                time, "world", "mm_base"));
        Eigen::Matrix4d mat = mm_config_->getTq0(), mat_nouse;
        Eigen::Matrix3d rot = mat.block(0, 0, 3, 3);
        Eigen::Quaterniond quaternion(rot);
        broadcaster_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()), 
                tf::Vector3(mat(0, 3), mat(1, 3), mat(2, 3))),
                time, "mm_base", "mani_0"));
        for(int i = 0; i < 6; ++i){
            mm_config_->getAJointTran(i, mm_state_.joint_p(i), mat, mat_nouse);
            rot = mat.block(0, 0, 3, 3);
            Eigen::Quaterniond quaternion1(rot);
            std::string frame1 = "mani_" + std::to_string(i);
            std::string frame2 = "mani_" + std::to_string(i + 1);
            broadcaster_.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(quaternion1.x(), quaternion1.y(), quaternion1.z(), quaternion1.w()), 
                    tf::Vector3(mat(0, 3), mat(1, 3), mat(2, 3))),
                    time, frame1, frame2));
        }
        
    }

    void ModelManager::odomCallback(const nav_msgs::OdometryConstPtr& odom){
        have_car_odom_ = true;
        Eigen::Vector3d new_car_p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        if((mm_state_.car_p - new_car_p).norm() > 1e-2){
            vis_his_traj(new_car_p.head(2));
        }
        mm_state_.feed_odom(odom);
        if(have_mani_odom_){
            mm_config_->visMM(vis_mm_pub_, "vis_mm_odom", 0, -0.9, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p, gripper_state_);
            mm_config_->visMMCheckBall(vis_mm_check_ball_pub_, "vis_mm_check_ball", 0, 0.7, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p);
        }
        his_traj_.push_back(Eigen::Vector2d(mm_state_.car_p(0), mm_state_.car_p(1)));
    }

    void ModelManager::jointStateCallback(const sensor_msgs::JointState::ConstPtr& state){
        have_mani_odom_ = true;
        mm_state_.feed_joint(state);
        if(have_car_odom_){
            mm_config_->visMM(vis_mm_pub_, "vis_mm_odom", 0, -0.9, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p, gripper_state_);
            mm_config_->visMMCheckBall(vis_mm_check_ball_pub_, "vis_mm_check_ball", 0, 0.7, Eigen::Vector3d(mm_state_.car_p(0), mm_state_.car_p(1), mm_state_.car_yaw), mm_state_.joint_p);
        }
    }

    void ModelManager::gripperStateCallback(const std_msgs::Bool::ConstPtr& state){
        if(gripper_state_ != state->data || (!have_gripper_state_)){
            have_gripper_state_ = true;
            gripper_state_ = state->data;
            mm_config_->setGripperPoint(gripper_state_);
        }
    }

    void ModelManager::vis_his_traj(Eigen::Vector2d pt){
        geometry_msgs::Point vis_pt;
        vis_pt.x = pt(0);
        vis_pt.y = pt(1);
        vis_pt.z = 0.0;
        his_traj_sphere_.points.push_back(vis_pt);
        his_traj_line_strip_.points.push_back(vis_pt);
        vis_his_traj_pub_.publish(his_traj_sphere_);
        if(his_traj_line_strip_.points.size() > 1)
            vis_his_traj_pub_.publish(his_traj_line_strip_);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "model_vis");
    ros::NodeHandle nh("~");
    model_vis::ModelManager mm;
    mm.init(nh);
    ros::spin();
    return 0;
}