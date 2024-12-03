#ifndef _KINO_ASTAR_H_
#define _KINO_ASTAR_H_

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <algorithm>

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>

#include "traj_utils/plan_container.hpp"
#include "plan_env/grid_map.h"
#include "sample_mani_RRT.h"

namespace remani_planner{

#define inf 1 >> 30

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class PathNode {
public:
  /* -------------------- */
  Eigen::Vector2i index;
  int yaw_idx;
  /* --- the state is x y yaw */
  Eigen::Vector3d state;
  double g_score, f_score;
  double penalty_score;
  /* control input should be steer and arc */
  Eigen::Vector2d input;
  PathNode* parent;
  char node_state;
  int singul = 0;
  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
public:
  template <class NodePtr>
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash_astar : std::unary_function<T, size_t> {
std::size_t operator()(T const& matrix) const {
	size_t seed = 0;
	for (long int i = 0; i < matrix.size(); ++i) {
	auto elem = *(matrix.data() + i);
	seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}
	return seed;
}
};

template <class NodePtr>
class NodeHashTable {
private:
/* data */
std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash_astar<Eigen::Vector3i>> data_3d_;
public:
NodeHashTable(/* args */) {}
~NodeHashTable() {}
void insert(Eigen::Vector2i idx, int yaw_idx, NodePtr node){
	data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), yaw_idx), node));
}

NodePtr find(Eigen::Vector2i idx, int yaw_idx) {
	auto iter = data_3d_.find(Eigen::Vector3i(idx[0], idx[1], yaw_idx));
	return iter == data_3d_.end() ? NULL : iter->second;
}

void clear() {
	data_3d_.clear();
}
};


class KinoAstar{
  private:
    ros::NodeHandle nh_;
	  int manipulator_dof_;
    int try_astar_times_;

    std::vector<PathNodePtr> path_node_pool_;
    NodeHashTable<PathNodePtr> expanded_nodes_;
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

    std::shared_ptr<GridMap> sdf_map_;
   
    double inv_yaw_resolution_;
    double lambda_heu_;
    int allocate_num_;
    double max_search_time_;
    double traj_forward_penalty_;
    double traj_back_penalty_;
    double traj_gear_switch_penalty_;
    double traj_steer_penalty_;
    double traj_steer_change_penalty_;
    
    double oneshot_check_len_;
    double grid_interval_;
    int check_num_;
    double oneshot_range_;
    double sample_time_;
    double planning_horizon_;

    double time_resolution_;

    int use_node_num_;
    int iter_num_;
    double non_siguav_;
    
    double max_vel_;
    double max_acc_;
    double max_steer_;
    double max_arc_;
    Eigen::Vector2i map_size_;

    double min_turning_radius_;
    double curvatureDisCoe_;

    // ompl::base::StateSpacePtr shotptr_;
    std::vector<ompl::base::StateSpacePtr> shotptr_list_;
    int shotptrindex_;

    bool is_shot_succ_= false;

    std::vector<double> shot_lengthList;
    std::vector<double> shot_timeList;
    std::vector<int> shotindex;
    std::vector<int> shot_SList;

    ros::Publisher frontend_path_pub_, local_start_goal_pub_;
  
    double mobile_base_wheel_base_;
    std::vector<PathNodePtr> path_nodes_;
    std::vector<CarFlatTrajData> flat_trajs_;
    std::vector<Eigen::Vector3d> sample_trajs_;
    Eigen::Vector4d start_state_; // x y yaw vel
    Eigen::Vector2d start_ctrl_; // 
    Eigen::Vector4d end_state_; // x y yaw vel
    double totalTrajTime;
    bool has_path_ = false;

  public:
    mani_sample::SampleMani::Ptr mani_sample_;
    std::shared_ptr<MMConfig> mm_config_;
    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4, START_COLLISION = 5, GOAL_COLLISION = 6};
    KinoAstar(){}
    ~KinoAstar();
    void setParam(ros::NodeHandle& nh, const std::shared_ptr<GridMap> &env, const std::shared_ptr<MMConfig> &mm_config);
    // void init();
    void reset();
    void visPath(std::vector<std::vector<Eigen::Vector3d>> path, bool pt);
    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ );
    void displayLocalStartGoal(ros::Publisher &pub, const vector<Eigen::VectorXd> &list);
    int KinoAstarSearchAndGetSimplePath(const Eigen::VectorXd &start_pos, const Eigen::VectorXd &start_vel, const double start_yaw, const int start_singul, const bool start_gripper,
                                        const Eigen::VectorXd &end_pos, const Eigen::VectorXd &end_vel, double end_yaw, const bool end_gripper, const Eigen::Vector2d &init_ctrl, const int continous_failures_count,
                                        std::vector<std::vector<Eigen::VectorXd>> &simple_path_container, std::vector<std::vector<double>> &yaw_list_container, 
                                        std::vector<int> &singul_container, std::vector<Eigen::VectorXd> &t_list_container);
    void simplifyRoute(const std::vector<Eigen::Vector3d> &node_path, 
                       const std::vector<Eigen::Vector2d> &node_input, 
                       std::vector<Eigen::Vector3d> &SampleList);
    int search(Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state, const Eigen::Vector2d &init_ctrl);
    int yawToIndex(const double &yaw);
    double normalize_angle(const double &angle);
    inline int getSingularity(const double &vel);
    inline double getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
    bool is_shot_sucess(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2, int last_dir);
    double computeShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2,const int shotptrind,
                           std::vector<Eigen::Vector3d> &path_list,
                           double& len);
    void retrievePath(const PathNodePtr &end_node);
    void stateTransit(const Eigen::Vector3d &state0, const Eigen::Vector2d &ctrl_input, Eigen::Vector3d &state1);

    void getKinoNode();
    void getSampleTrajs();
    void getTrajsWithTime();
    double evaluateCurvatureWeightedDistance(const Eigen::Vector2d &state1, const Eigen::Vector2d &state2, const double &yaw1, const double &yaw2);
    double evaluateDuration(const double &length, const double &startV, const double &endV);
    double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV);
    void getFlatState(const Eigen::Vector4d &state, const Eigen::Vector2d &control_input,const int &singul,Eigen::MatrixXd &flat_state);
    Eigen::Vector3d evaluatePos(const double &t);

    typedef shared_ptr<KinoAstar> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace remani_planner
#endif