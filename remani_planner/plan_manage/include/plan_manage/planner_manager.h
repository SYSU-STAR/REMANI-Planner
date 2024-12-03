
#pragma once

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.hpp>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include "traj_utils/plan_container.hpp"
#include <ros/ros.h>
#include <plan_manage/planning_visualization.h>
#include "traj_utils/poly_traj_utils.hpp"
#include <std_msgs/Bool.h>

namespace remani_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class MMPlannerManager
  {
  public:
  
    MMPlannerManager();
    ~MMPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /* main planning interface */
    bool reboundReplan(
        const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc, 
        const Eigen::VectorXd &start_jerk, const double start_yaw, const int start_singul, const bool start_gripper, const double trajectory_start_time, 
        const Eigen::VectorXd &end_pt, const Eigen::VectorXd &end_vel ,const Eigen::VectorXd &end_acc, double end_yaw, const bool local_target_gripper,
        const bool flag_polyInit, const bool flag_randomPolyTraj,
        const bool have_local_traj, double &init_time, double &opt_time);
    bool computeInitReferenceState(const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, 
                                    const Eigen::VectorXd &start_acc, const Eigen::VectorXd &start_jerk,
                                    const double start_yaw, const int start_singul, const bool start_gripper,
                                    const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
                                    const Eigen::VectorXd &local_target_acc, const double local_target_yaw, const bool local_target_gripper,
                                    std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container, 
                                    std::vector<int> &singul_container,
                                    const bool flag_polyInit, const int continous_failures_count);
    bool planGlobalTrajWaypoints(
        const Eigen::VectorXd &start_pos, const double start_yaw, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
        const std::vector<Eigen::VectorXd> &waypoints, const double end_yaw, const Eigen::VectorXd &end_vel, const Eigen::VectorXd &end_acc);
    void getLocalTarget(
        const double planning_horizen, const Eigen::VectorXd &start_pt, const double &start_yaw,
        const Eigen::VectorXd &global_end_pt, const double global_end_yaw,
        Eigen::VectorXd &local_target_pos, Eigen::VectorXd &local_target_vel,Eigen::VectorXd &local_target_acc, bool &reach_horizon);
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool EmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul);

    PlanParameters pp_;
    // LocalTrajData local_data_;
    // GlobalTrajData global_data_;
    std::shared_ptr<GridMap> grid_map_;
    TrajContainer traj_container_;
    
    // ros::Publisher obj_pub_; //zx-todo

    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    std::shared_ptr<MMConfig> mm_config_;

    bool start_flag_, reach_flag_;
    ros::Time global_start_time_;
    ros::Publisher destory_cmd_pub_;
    double start_time_, reach_time_, average_plan_time_;
    std::vector<double> total_time_;
    std::vector<double> init_time_;
    std::vector<double> opt_time_;

  private:
    /* main planning algorithms & modules */
    
    PlanningVisualization::Ptr visualization_;

    int continous_failures_count_{0};

  public:
    typedef unique_ptr<MMPlannerManager> Ptr;

    // !SECTION
  };
} // namespace remani_planner