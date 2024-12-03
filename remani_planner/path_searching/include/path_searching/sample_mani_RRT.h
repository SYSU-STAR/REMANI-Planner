#ifndef _SAMPLE_MANI_H_
#define _SAMPLE_MANI_H_

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
#include "path_searching/rrt.h"
#include <fstream>

namespace mani_sample {

  class ManiPathNode{
    public:
    enum NODE_STATE
    {
      EXPAND,
      NOT_EXPAND,
      IN_TREE,
      IN_ANTI_TREE,
      COLLISION
    };

    int index;
    Eigen::VectorXd state;
    ManiPathNode* parent;
    double g_score;
    NODE_STATE node_state;
    std::map<string, ManiPathNode*> children;

    ManiPathNode(){
      parent = nullptr;
      node_state = NOT_EXPAND;
    } 
    ManiPathNode(int dof){
      parent = nullptr;
      node_state = NOT_EXPAND;
      state = Eigen::VectorXd::Zero(dof);
    }
    ~ManiPathNode(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef ManiPathNode* ManiPathNodePtr;

  class SampleMani{

    private:
    ManiPathNodePtr end_node_;
    std::map<string, ManiPathNodePtr> node_pool_;
    std::vector<Eigen::Vector3d> car_state_list_;
    std::vector<Eigen::Vector3d> car_state_list_check_;
    std::vector<double> t_list_;

    
    bool have_path_;
    Eigen::VectorXd min_joint_pos_, max_joint_pos_;
    double max_joint_vel_, max_joint_acc_;
    double self_safe_margin_, safe_margin_mani_;
    double mobile_base_check_radius_;
    int manipulator_dof_, mobile_base_dof_, traj_dim_;
    double mani_thickness_;
    double angle_res_;
    int max_index_, tree_max_index_, tree_min_index_;
    int tree_count_, anti_tree_count_;
    int check_num_;
    double goal_rate_;
    int max_loop_num_;
    double max_mani_search_time_;
    Eigen::Matrix3d phi_; // state transit matrix
    Eigen::Matrix4d T_q_0_;
    std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
    std::mt19937 goal_gen_;
    std::mt19937 state_gen_;
    std::mt19937 node_gen_;

    bool checkcollision(const ManiPathNodePtr& cur_state, const ManiPathNodePtr& next_state);
    int doubleIdx2int(double idx);
    Eigen::VectorXd idx2coord(const Eigen::VectorXi &s);
    Eigen::VectorXi coord2idx(const Eigen::VectorXd &s);
    double calAngleErr(double angle1, double angle2);
    bool feasibleCheck(ManiPathNodePtr &x1, ManiPathNodePtr &x2);
    double estimateHeuristic(ManiPathNodePtr &x1, ManiPathNodePtr &x2);
    string calculateValue(int &idx, const Eigen::VectorXd &state);
    string calculateValue(ManiPathNodePtr &x);
    ManiPathNodePtr initNode(int idx, const Eigen::VectorXd &s);
    ManiPathNodePtr getSampleNode();
    ManiPathNodePtr getNearestNode(ManiPathNodePtr &x, bool dir);
    ManiPathNodePtr extendNode(ManiPathNodePtr &q_near, ManiPathNodePtr &q_rand, bool dir);
    void oneShot(ManiPathNodePtr &q);
    void trajShot(ManiPathNodePtr &q);
    void allShot(ManiPathNodePtr &q);
    void linkNode(ManiPathNodePtr &parent, ManiPathNodePtr &child);
    void expandGscore(ManiPathNodePtr &p);
    void adjustTree(ManiPathNodePtr &q_new, bool dir);
    void organizeTree();
    void organizeTree(ManiPathNodePtr &q);
    void clearSubTree(ManiPathNodePtr &q);
    void mergeTrees(const ManiPathNodePtr &q1, const ManiPathNodePtr &q2);
    bool fullStateRepair();

    public:
    remani_planner::RrtPlanning::Ptr rrt_plan_;
    std::shared_ptr<remani_planner::MMConfig> mm_config_;
    SampleMani():
    goal_gen_(std::random_device{}()),
    state_gen_(std::random_device{}()),
    node_gen_(std::random_device{}())
    {};
    ~SampleMani(){
      for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
        delete it->second;
      }
      node_pool_.clear();
    }

    bool search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state);
    void setParam(ros::NodeHandle& nh, const std::shared_ptr<remani_planner::MMConfig> &mm_config);
    void init(const std::vector<Eigen::Vector3d> &car_state_list, const std::vector<Eigen::Vector3d> &car_state_list_check, const std::vector<double> &t_list);
    void reset();
    bool getTraj(std::vector<Eigen::VectorXd> &traj);
    int getTreeNum(){return tree_count_;}
    double getCost();
    bool sampleManiSearch(const bool astar_succ, const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state,
                    const std::vector<Eigen::Vector3d> &car_state_list, const std::vector<Eigen::Vector3d> &car_state_list_check, 
                    const std::vector<double> &t_list, const std::vector<int> &singul_container, const int start_singul,// size = t_list.size()
                    std::vector<std::vector<Eigen::VectorXd>> &simple_path_container, std::vector<int> &singul_container_new,
                    std::vector<std::vector<double>> &yaw_list_container, std::vector<Eigen::VectorXd> &t_list_container);
    typedef shared_ptr<SampleMani> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
};

#endif
