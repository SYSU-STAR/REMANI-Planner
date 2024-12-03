#include "mm_controller/controller_utils.hpp"
#include <algorithm> 

namespace MMController{

void Trajectory_Data_t::feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg){
    const quadrotor_msgs::PolynomialTraj &traj = *pMsg;
    if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ADD){
        if ((int)traj.trajectory_id < 1){
            ROS_ERROR("[MMCtrl Traj] The trajectory_id must start from 1");
            return;
        }
        if(traj.trajectory_id == 1){
            singul_traj_data.clearSingulTraj();
            singul_traj_data.start_time_ros = pMsg->header.stamp;
            singul_traj_data.start_time = pMsg->header.stamp.toSec();
            total_traj_start_time_ros = ros::Time::now();
            total_traj_start_time = ros::Time::now().toSec();
        }
        
        poly_traj::Trajectory<7> polyTraj;
        double t_total = 0;
        for (auto &piece : traj.trajectory){
            polyTraj.emplace_back(piece.duration, Eigen::Map<const Eigen::MatrixXd>(&piece.data[0], piece.num_dim, (piece.num_order + 1)), traj.singul);
            t_total += piece.duration;
        }
        singul_traj_data.addSingulTraj(polyTraj, singul_traj_data.start_time + singul_traj_data.duration);
        total_traj_end_time = total_traj_start_time + singul_traj_data.duration;
        exec_traj = 1;
    }else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ABORT){
        ROS_WARN("[MMCtrl] Aborting the trajectory.");
        total_traj_start_time_ros = ros::Time(0);
        total_traj_end_time_ros = ros::Time(0);
        singul_traj_data.clearSingulTraj();
        total_traj_start_time = 0.0;
        total_traj_end_time = 0.0;
        exec_traj = -1;
    }else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_WARN_IMPOSSIBLE){
        total_traj_start_time_ros = ros::Time(0);
        total_traj_end_time_ros = ros::Time(0);
        total_traj_start_time = 0.0;
        total_traj_end_time = 0.0;
        singul_traj_data.clearSingulTraj();
        exec_traj = -1;
    }
}

}
