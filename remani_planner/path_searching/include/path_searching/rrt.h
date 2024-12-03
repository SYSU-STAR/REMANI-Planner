#ifndef _MM_CAR_FULL_RRT_PLANNING_
#define _MM_CAR_FULL_RRT_PLANNING_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <time.h>
#include <random>
#include <cstdint>
#include "plan_env/grid_map.h"
#include "mm_config/mm_config.hpp"
#include <fstream>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>

namespace remani_planner{

  class PathNodeRRT{
    public:
    enum NODE_STATE
    {
      TO_EXPAND,
      EXPAND,
      IN_TREE,
      IN_ANTI_TREE,
      COLLISION
    };

    int singul;
    int layer;
    string index;
    Eigen::VectorXd state;
    double yaw;
    PathNodeRRT* parent;
    double g_score, f_score;
    NODE_STATE node_state;
    std::map<string, PathNodeRRT*> children;

    PathNodeRRT(){
      parent = nullptr;
      node_state = TO_EXPAND;
    } 
    PathNodeRRT(int dof){
      parent = nullptr;
      node_state = TO_EXPAND;
      state = Eigen::VectorXd::Zero(dof);
    }
    // PathNodeRRT(Eigen::VectorXd pre_state){
    //   parent = nullptr;
    //   node_state = NOT_EXPAND;
    //   state = pre_state;
    // }
    ~PathNodeRRT(){}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef PathNodeRRT* PathNodeRRTPtr;

  class RrtPlanning{

    private:
    PathNodeRRTPtr end_node_;
    std::map<string, PathNodeRRTPtr> node_pool_;
    
    bool have_path_;
    Eigen::VectorXd min_joint_pos_, max_joint_pos_;
    double max_joint_vel_, max_joint_acc_;
    double max_vel_, max_acc_;
    double self_safe_margin_, safe_margin_mani_;
    int mobile_base_dof_, manipulator_dof_, traj_dim_;
    double mani_thickness_;
    double angle_res_;
    double c_max_, c_min_;
    Eigen::VectorXd sample_start_, sample_end_;
    double sample_start_radius_, sample_end_radius_;
    double max_sample_time_;
    int max_index_, tree_max_index_;
    int tree_count_, anti_tree_count_;
    int check_num_;
    double goal_rate_;
    int max_loop_num_;
    std::vector<double> max_size_, min_size_;
    Eigen::Matrix4d T_q_0_;
    std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
    std::mt19937 random_gen_;
    std::uniform_real_distribution<double> random_dis_;
    std::normal_distribution<double> norm_dis_;
    ompl::base::StateSpacePtr dubins_curve_;
    double time_resolution_;

    bool checkcollision(PathNodeRRTPtr& cur_state, PathNodeRRTPtr& next_state);
    bool checkcollision(PathNodeRRTPtr& cur_state, const Eigen::VectorXd& next_state, const double next_yaw);
    bool checkcollision(PathNodeRRTPtr& cur_state);
    // bool checkcollision(Eigen::VectorXd cur_state);
    void expandGscore(PathNodeRRTPtr q);
    double calAngleErr(double angle1, double angle2);
    double estimateHeuristic(PathNodeRRTPtr &x1, PathNodeRRTPtr &x2);
    double estimateHeuristic(const Eigen::VectorXd& s1, const double& yaw1, const Eigen::VectorXd& s2, const double& yaw2);
    // double estimateHeuristic(const Eigen::VectorXd &s1, const Eigen::VectorXd &s2);
    PathNodeRRTPtr initNode(const Eigen::VectorXd &s, const double yaw);
    void sample(Eigen::VectorXd &s_state, double &s_yaw);
    PathNodeRRTPtr near(const Eigen::VectorXd &s, const double yaw, bool flag);
    PathNodeRRTPtr steer(PathNodeRRTPtr &q_near, const Eigen::VectorXd &s, const double s_yaw, double step_time);
    void rewire(PathNodeRRTPtr q_new, double near_time);
    void linkNode(PathNodeRRTPtr &parent, PathNodeRRTPtr &child);
    void mergeTree(PathNodeRRTPtr &s1, PathNodeRRTPtr &s2);
    double getTime(PathNodeRRTPtr &pre_node, PathNodeRRTPtr &cur_node);
    string calculateValue(PathNodeRRTPtr &q);
    string calculateValue(const Eigen::VectorXd &s, const double yaw);

    public:
    std::shared_ptr<MMConfig> mm_config_;
    RrtPlanning():
    random_gen_(std::random_device{}()),
    random_dis_(0.0, 1.0),
    norm_dis_(0.0, 1.0){};
    ~RrtPlanning(){
      for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
        delete it->second;
      }
      node_pool_.clear();
    }

    int RRTSearchAndGetSimplePath(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list, 
                                  const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
                                  const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list, const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
                                  const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list,
                                  std::vector<Eigen::VectorXd>& path, std::vector<double>& yaw_list, std::vector<double>& t_list);
    bool search(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list, 
                const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
                const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list, const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
                const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list);
    void setParam(ros::NodeHandle& nh, const std::shared_ptr<MMConfig> &mm_config);
    void init(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<Eigen::VectorXd>& end_pt_list);
    void reset();
    bool smooth(PathNodeRRTPtr smooth_node);
    bool getTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> & yaw_list, std::vector<double> &t_list);
    void smoothTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> &yaw_list, std::vector<double> &t_list);
    int getPathLen();
    void displayPath();
    int getTreeNum(){return tree_count_;}
    double getCost();
    bool getLayer(int& start_layer, int& end_layer);

    typedef shared_ptr<RrtPlanning> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
};

#endif
