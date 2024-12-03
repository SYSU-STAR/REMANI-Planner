#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <fstream>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.hpp>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/planning_visualization.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/Assignment.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iostream>
using std::vector;

namespace remani_planner
{

  class REMANIReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      // WAIT_GRIPPER,
      EMERGENCY_STOP,
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2
    };
    
    /* planning utils */
    MMPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    int wpt_id_;
    double no_replan_thresh_, replan_thresh_;
    // double waypoints_[50][4];
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> waypoints_;
    std::vector<double> waypoints_yaw_;
    std::vector<bool> waypoint_gripper_close_;
    bool gripper_flag_; // 是否需要夹爪操作
    int waypoint_num_;
    double planning_horizen_;
    double emergency_time_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;
    int replan_fail_time_;
    double time_for_gripper_;
    bool global_plan_;

    int mobile_base_dim_, manipulator_dim_, traj_dim_;
    double mobile_base_non_singul_vel_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::VectorXd mm_state_pos_, mm_state_vel_, mm_state_acc_, init_state_; // odometry state
    bool gripper_state_, rcv_gripper_state_;
    int mm_car_singul_;
    Eigen::Quaterniond mm_car_orient_;
    double mm_car_yaw_, mm_car_yaw_rate_;

    Eigen::VectorXd start_pos_, start_vel_, start_acc_, start_jer_; // start state
    int start_singul_;
    double start_yaw_, end_yaw_;
    Eigen::VectorXd end_pt_;                                       // goal state
    Eigen::VectorXd local_target_pt_, local_target_vel_, local_target_acc_;                     // local target state
    int local_target_singul_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;
    bool try_plan_after_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, joint_state_sub_, gripper_state_sub_, trigger_sub_, assignment_sub_;
    ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, gripper_cmd_pub_, map_state_pub_;

    ros::Publisher reached_pub_, start_pub_;

    ros::Time t_last_Astar_;

    int map_state_;

    std::vector<double> init_time_list_;
    std::vector<double> opt_time_list_;
    std::vector<double> total_time_list_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);           // front-end and back-end method
    bool callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul); // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init);
    
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool planNextWaypoint(const Eigen::VectorXd next_wp, const double nect_yaw);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg);
    void gripperCallback(const std_msgs::Bool::ConstPtr &msg);
    void sendPolyTrajROSMsg();
    bool frontEndPathSearching();
    bool checkCollision();

  public:
    REMANIReplanFSM(/* args */)
    {
    }
    ~REMANIReplanFSM();


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace remani_planner

#endif