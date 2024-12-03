#include "path_searching/kino_astar.h"
#include <sstream>
#include <plan_env/grid_map.h>

using namespace std;
using namespace Eigen;

namespace remani_planner{

void KinoAstar::setParam(ros::NodeHandle& nh, const std::shared_ptr<GridMap> &env, const std::shared_ptr<MMConfig> &mm_config){
  sdf_map_ = env;
  Eigen::Vector3i map_size;
  sdf_map_->getVoxelNum(map_size);
  map_size_ = map_size.head(2);
  
  mm_config_ = mm_config;
  max_vel_ = mm_config_->getBaseMaxVel();
  max_acc_ = mm_config_->getBaseMaxAcc();
  manipulator_dof_ = mm_config_->getManiDof();
  mobile_base_wheel_base_ = mm_config_->getBaseWheelBase();

  double yaw_resolution;
  nh.param("search/yaw_resolution", yaw_resolution, 0.3);
  inv_yaw_resolution_ = 1.0 / yaw_resolution;
  nh.param("search/lambda_heu", lambda_heu_, 5.0);
  nh.param("search/max_seach_time", max_search_time_, 0.1);
  nh.param("search/traj_forward_penalty", traj_forward_penalty_, 1.0);
  nh.param("search/traj_back_penalty", traj_back_penalty_, 1.0);
  nh.param("search/traj_gear_switch_penalty", traj_gear_switch_penalty_, 15.0);
  nh.param("search/traj_steer_penalty", traj_steer_penalty_, 0.5);
  nh.param("search/traj_steer_change_penalty", traj_steer_change_penalty_, 0.0);
  nh.param("search/max_arc", max_arc_, 0.9);
  nh.param("search/oneshot_check_len", oneshot_check_len_, 0.2);
  
  nh.param("search/check_num", check_num_, 5);
  nh.param("search/oneshot_range", oneshot_range_, 5.0);
  nh.param("search/sample_time", sample_time_, 0.1);
  nh.param("search/min_turning_radius", min_turning_radius_, 0.1);
  nh.param("search/curvatureDisCoe", curvatureDisCoe_, 0.4);
  nh.param("mm/mobile_base_non_singul_vel", non_siguav_, 0.01);

  double dist_resolution;
  nh.param("search/dist_resolution", dist_resolution, -1.0);

  nh.param("search/allocate_num", allocate_num_, 100000);
  nh.param("search/try_astar_times", try_astar_times_, 20);
  nh.param("fsm/planning_horizon", planning_horizon_, 10.0);

  bool global_plan;
  nh.param("fsm/global_plan", global_plan, false);
  if(global_plan){
    planning_horizon_ = 1000.0;
  }

  frontend_path_pub_ = nh.advertise<visualization_msgs::Marker>("kinoastar/path", 10);
  local_start_goal_pub_ = nh.advertise<visualization_msgs::MarkerArray>("kinoastar/local_start_goal", 10);

  use_node_num_ = 0;
  iter_num_ = 0;
  has_path_ = false;
  shotptrindex_ = -1;
  shotptr_list_.clear();

  // shotptr_list_.push_back(std::make_shared<ompl::base::DubinsStateSpace>(0.2));
  // shotptr_list_.push_back(std::make_shared<ompl::base::DubinsStateSpace>(0.1));
  // shotptr_list_.push_back(std::make_shared<ompl::base::DubinsStateSpace>(0.05));
  
  shotptr_list_.push_back(std::make_shared<ompl::base::ReedsSheppStateSpace>(0.4));
  shotptr_list_.push_back(std::make_shared<ompl::base::ReedsSheppStateSpace>(0.2));
  shotptr_list_.push_back(std::make_shared<ompl::base::ReedsSheppStateSpace>(0.1));

  mani_sample_.reset(new mani_sample::SampleMani);
  mani_sample_->setParam(nh, mm_config_);
  time_resolution_ = dist_resolution / max_vel_;
  max_steer_ = std::atan(mobile_base_wheel_base_ / min_turning_radius_);

  reset();
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; ++i){
    path_node_pool_[i] = new PathNode;
  }
  grid_interval_ = sdf_map_->getResolution();
}

void KinoAstar::reset(){
  expanded_nodes_.clear();
  path_nodes_.clear();
  flat_trajs_.clear();
  sample_trajs_.clear();
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++){
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  shotptrindex_ = -1;
  is_shot_succ_ = false;
  has_path_ = false;
}

KinoAstar::~KinoAstar(){
  for (int i = 0; i < allocate_num_; i++){
    delete path_node_pool_[i];
  }
}

int KinoAstar::KinoAstarSearchAndGetSimplePath(const Eigen::VectorXd &start_pos, const Eigen::VectorXd &start_vel, const double start_yaw, const int start_singul, const bool start_gripper,
                                                const Eigen::VectorXd &end_pos, const Eigen::VectorXd &end_vel, double end_yaw, const bool end_gripper, const Eigen::Vector2d &init_ctrl, const int continous_failures_count,
                                                std::vector<std::vector<Eigen::VectorXd>> &simple_path_container, std::vector<std::vector<double>> &yaw_list_container, 
                                                std::vector<int> &singul_container, std::vector<Eigen::VectorXd> &t_list_container){
  reset();
  simple_path_container.clear();
  yaw_list_container.clear();
  singul_container.clear();
  t_list_container.clear();
  std::vector<int> singul_container_temp;
  
  Eigen::VectorXd start_state, end_state;
  start_state.resize(4 + manipulator_dof_);
  start_state.head(2) = start_pos.head(2);
  start_state(2) = start_yaw;
  start_state(3) = start_vel.head(2).norm() * start_singul;
  start_state.tail(manipulator_dof_) = start_pos.tail(manipulator_dof_);

  end_state.resize(4 + manipulator_dof_);
  end_state.head(2) = end_pos.head(2);
  end_state(2) = end_yaw;
  end_state(3) = 0.0;
  end_state.tail(manipulator_dof_) = end_pos.tail(manipulator_dof_);

  mm_config_->visMM(local_start_goal_pub_, "start", 0, 0.8, start_state.head(3), start_state.tail(manipulator_dof_), start_gripper);
  mm_config_->visMM(local_start_goal_pub_, "gaol", 1, 0.8, end_state.head(3), end_state.tail(manipulator_dof_), end_gripper);

  std::cout << "continous_failures_count: " << continous_failures_count << "\n";
  bool start_goal_is_close = (start_pos - end_pos).head(2).norm() < 8e-2 && (start_pos - end_pos).tail(manipulator_dof_).norm() > 1e-1;
  if(continous_failures_count < try_astar_times_ && (!start_goal_is_close) /*&& false*/){
    // ROS_WARN("ASTAR!");
    search(start_state, end_state, init_ctrl);
  }else{
    // ROS_WARN("FAILED TOO MANY TIMES, use rrt!");
    has_path_ = false;
  }
  sdf_map_->resetAstarBuffer();
  bool sample_succ;
  
  if(has_path_){
    getKinoNode();
    double basetime = 0.0;
    int piece_singul_num = flat_trajs_.size();
    std::vector<Eigen::VectorXd> pieceTimes;
    pieceTimes.resize(piece_singul_num);
    Eigen::VectorXi eachTrajNums;
    eachTrajNums.resize(piece_singul_num);

    Eigen::VectorXd state;
    state.resize(2 + manipulator_dof_);
    std::vector<std::vector<Eigen::Vector3d>> car_statelist_disp;
    std::vector<std::vector<Eigen::Vector3d>> car_statept_disp;
    std::vector<Eigen::Vector3d> car_statelist, car_statelist_check;
    Eigen::VectorXd piecetime_all;
    std::vector<double> init_t_list;
    Eigen::Vector3d pos;
    double res_time;
    for(int i = 0; i < piece_singul_num; i++){
      CarFlatTrajData kino_traj = flat_trajs_.at(i);
      std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
      int piece_nums;
      double initTotalduration = 0.0;
      for(const auto pt : pts){
        initTotalduration += pt[2];
      }

      piece_nums = std::max(int(initTotalduration / time_resolution_ + 0.5), 2);
      double timePerPiece = initTotalduration / piece_nums;
      Eigen::VectorXd piecetime;
      piecetime.resize(piece_nums);
      piecetime.setConstant(timePerPiece);
      res_time = 0;
      
      for(int j = 0; j < piece_nums; j++){
        init_t_list.push_back(timePerPiece);
        singul_container_temp.push_back(kino_traj.singul);
        pos = evaluatePos(basetime + res_time);
        car_statelist.push_back(pos);
        for(int k = 0; k < check_num_; ++k){
          double t = basetime + res_time + 1.0 * k / check_num_ * timePerPiece;
          pos = evaluatePos(t);
          car_statelist_check.push_back(pos); // FIXME inner pts
        }
        res_time += timePerPiece;
      }
      
      if(i == piece_singul_num - 1){
        pos = evaluatePos(basetime + res_time - 1.0e-2);
        car_statelist.push_back(pos);
        car_statelist_check.push_back(pos);
      }
      basetime += initTotalduration;
    }

    car_statept_disp.push_back(car_statelist);
    car_statelist_disp.push_back(car_statelist_check);
    visPath(car_statept_disp, true);
    visPath(car_statelist_disp, false);

    std::vector<Eigen::VectorXd> final_state;
    std::vector<double> final_yaw_list, final_t_list;
    sample_succ = mani_sample_->sampleManiSearch(true,
        start_pos.tail(manipulator_dof_), end_pos.tail(manipulator_dof_), 
        car_statelist, car_statelist_check, // x, y, yaw
        init_t_list, singul_container_temp, start_singul,
        simple_path_container, singul_container,
        yaw_list_container, t_list_container);
  }else{
    std::vector<Eigen::Vector3d> car_statelist, car_statelist_check;
    std::vector<double> init_t_list;
    Eigen::Vector3d start_state_temp, end_state_temp;
    start_state_temp << start_pos(0), start_pos(1), start_yaw;
    end_state_temp << end_pos(0), end_pos(1), end_yaw;
    car_statelist.push_back(start_state_temp);
    
    Eigen::Vector3d state_temp;
    for(int i = 0; i < check_num_; ++i){
      state_temp = start_state_temp + (end_state_temp - start_state_temp) * (double)i / double(check_num_);
      car_statelist_check.push_back(state_temp);
    }
    car_statelist.push_back(end_state_temp);
    car_statelist_check.push_back(end_state_temp);
    init_t_list.push_back((start_state_temp - end_state_temp).head(2).norm() / max_vel_);
    singul_container_temp.push_back(1);
    sample_succ = mani_sample_->sampleManiSearch(false,
        start_pos.tail(manipulator_dof_), end_pos.tail(manipulator_dof_), 
        car_statelist, car_statelist_check, // x, y, yaw
        init_t_list, singul_container_temp, start_singul,
        simple_path_container, singul_container,
        yaw_list_container, t_list_container);
  }

  double path_length = 0.0, length_temp;
  bool achieve_hori = false;
  for(unsigned int trajid = 0; trajid < simple_path_container.size(); ++trajid){
    for(unsigned int i = 0; i < simple_path_container[trajid].size() - 1; ++i){
      length_temp = (simple_path_container[trajid][i + 1] - simple_path_container[trajid][i]).head(2).norm();
      path_length += length_temp;
      if(path_length > planning_horizon_){
        simple_path_container.resize(trajid + 1);
        singul_container.resize(trajid + 1);
        yaw_list_container.resize(trajid + 1);
        t_list_container.resize(trajid + 1);
        int temp_size = i + 1;
        temp_size = std::min(std::max(2, temp_size), (int)(t_list_container[trajid].size()));
        simple_path_container[trajid].resize(temp_size + 1);
        yaw_list_container[trajid].resize(temp_size + 1);
        Eigen::VectorXd time_temp = t_list_container[trajid];
        t_list_container[trajid] = time_temp.head(temp_size);
        achieve_hori = true;
        break;
      }
    }
    if(achieve_hori) break;
  }
  if(sample_succ){
    if(achieve_hori) return KinoAstar::REACH_HORIZON;
    else return KinoAstar::REACH_END;
  }else{
    return KinoAstar::NO_PATH;
  }
}

void KinoAstar::visPath(std::vector<std::vector<Eigen::Vector3d>> path, bool pt){
  if (frontend_path_pub_.getNumSubscribers() == 0){
    return;
  }

  int i = 0;
  vector<Eigen::Vector3d> list;

  Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
  double scale = 0.05 + (double)rand() / RAND_MAX / 10;

  for (auto block : path){
    list.clear();
    for (auto pt : block){
      // std::cout << pt.head(2).transpose() << std::endl;
      list.push_back(Eigen::Vector3d(pt(0), pt(1), 0.1));
    }
    // Eigen::Vector4d color(0.5,0.5,0,1);
    displayMarkerList(frontend_path_pub_, list, scale, color, i, pt); // real ids used: [ id ~ id+a_star_paths.size() ]
    i++;
  }
}

void KinoAstar::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                  Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
{
  visualization_msgs::Marker MarkerDelete;
  MarkerDelete.action = visualization_msgs::Marker::DELETEALL;
  pub.publish(MarkerDelete);

  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "world";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 2;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = 0.0;
    if(show_sphere)  sphere.points.push_back(pt);
    if(!show_sphere) line_strip.points.push_back(pt);
  }
  if(show_sphere) pub.publish(sphere);
  if(!show_sphere) pub.publish(line_strip);
}

void KinoAstar::displayLocalStartGoal(ros::Publisher &pub, const vector<Eigen::VectorXd> &list)
{
  visualization_msgs::Marker MarkerDelete;
  MarkerDelete.action = visualization_msgs::Marker::DELETEALL;
  pub.publish(MarkerDelete);

  visualization_msgs::Marker sphere;
  sphere.header.frame_id= "world";
  sphere.header.stamp= ros::Time::now();
  sphere.type = visualization_msgs::Marker::MESH_RESOURCE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.id = 0;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = 0;
  sphere.color.g = 1;
  sphere.color.b = 0;
  sphere.color.a = 1.0;
  sphere.scale.x = 0.2;
  sphere.scale.y = 0.2;
  sphere.scale.z = 0.2;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    sphere.points.push_back(pt);
  }
  pub.publish(sphere);
}

int KinoAstar::search(Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state, const Eigen::Vector2d &init_ctrl){
  bool isocc = false;
  bool initsearch = false;
  double start_time = ros::Time::now().toSec();
  int coll_type;
  if(mm_config_->checkcollision(start_state.head(3), start_state.tail(manipulator_dof_), false, coll_type)){
    ROS_WARN("KinoAstar: start (%d) is not free!", coll_type);
    return START_COLLISION;
  }

  if(mm_config_->checkcollision(end_state.head(3), end_state.tail(manipulator_dof_), false, coll_type)){
    ROS_WARN("KinoAstar: goal (%d) is not free!", coll_type);
    return GOAL_COLLISION;
  }

  start_state_ = start_state.head(4);
  start_ctrl_  = init_ctrl;
  end_state_   = end_state.head(4);
  
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state = start_state_.head(3);
  cur_node->index = sdf_map_->pos2dToIndex(start_state_.head(2));
  cur_node->yaw_idx = yawToIndex(start_state_(2));
  cur_node->g_score = 0.0;
  cur_node->input = Eigen::Vector2d(0.0, 0.0);
  cur_node->singul = getSingularity(start_state_(3));
  cur_node->f_score = lambda_heu_ * getHeu(cur_node->state, end_state_.head(3));
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;
  expanded_nodes_.insert(cur_node->index, yawToIndex(start_state_(2)) ,cur_node);
  if(cur_node->singul == 0){initsearch = true;}
  while (!open_set_.empty()){
    cur_node = open_set_.top();
    if((cur_node->state.head(2) - end_state_.head(2)).norm() < oneshot_range_ && initsearch){
      int last_dir;
      if(cur_node->parent != NULL){
        bool is_forward = Eigen::Vector2d(cos(cur_node->parent->state(2)), sin(cos(cur_node->parent->state(2)))).transpose() * (cur_node->state - cur_node->parent->state).head(2) > 0;
        last_dir = is_forward ? 1 : -1;
      }else{
        last_dir = 0;
      }
      is_shot_sucess(cur_node->state, end_state_.head(3), last_dir);
      if(is_shot_succ_){
        start_state = end_state;
        retrievePath(cur_node);
        has_path_ = true;
        return REACH_END;
      }
    }
    
    if(ros::Time::now().toSec() - start_time > max_search_time_){
      printf("\033[Kino Astar]: Kino Astar NO_PATH. Exceeds maximum time %lf s \n\033[0m", ros::Time::now().toSec() - start_time);
      has_path_ = false;
      return NO_PATH;
    }

    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;
    
    Eigen::Vector3d cur_state = cur_node->state;
    Eigen::Vector3d pro_state;
    Eigen::Vector2d ctrl_input;
    std::vector<Eigen::Vector2d> inputs;
    double res_steer = 0.5;
    double res_arc = 0.5;
    if(!initsearch){
      if(start_state_[3]>0){
        for(double arc = 1.0 * grid_interval_; arc <= 2.0 * grid_interval_ + 1e-3; arc += 1.0 * grid_interval_){
          if(fabs(arc) < 1.0e-2){
            continue;
          }
          ctrl_input << 0.0, arc; // only straight forward
          inputs.push_back(ctrl_input);
        }
      }else{
        for(double arc = -1.0 * grid_interval_; arc >= -2.0 * grid_interval_ - 1e-3; arc -= 1.0 * grid_interval_){
          if(fabs(arc) < 1.0e-2){
            continue;
          }
          ctrl_input << 0.0, arc; // only straight backward
          inputs.push_back(ctrl_input);
        }
      }
      initsearch = true;
    }else{
      for (double arc = -max_arc_; arc <= max_arc_ + 1e-3; arc += res_arc * max_arc_){
        if(fabs(arc) < 1.0e-2){
          continue;
        }
        for (double steer = -max_steer_; steer <= max_steer_ + 1e-3; steer += res_steer * max_steer_*1.0){
          if(fabs(steer) > max_steer_ * 0.9 && fabs(arc) > max_arc_ * 0.9){
            continue; // avoid little circle path
          }
          ctrl_input << steer, arc;
          inputs.push_back(ctrl_input);
        }
      }
    }
    for (auto& input : inputs){
      int singul;
      singul = getSingularity(input[1]);
      stateTransit(cur_state, input, pro_state);
      if(!sdf_map_->isInMap(Eigen::Vector2d(pro_state(0), pro_state(1)))){
        continue;
      }

      Eigen::Vector2i pro_id = sdf_map_->pos2dToIndex(pro_state.head(2));
      int pro_yaw_id = yawToIndex(pro_state(2));
      PathNodePtr pro_node;
      pro_node = expanded_nodes_.find(pro_id, pro_yaw_id);

      if(pro_node != NULL && pro_node->node_state == IN_CLOSE_SET){continue;}

      Eigen::Vector2i diff = pro_id - cur_node->index;
      int diff_yaw = pro_yaw_id - cur_node->yaw_idx;
      if (diff.norm() == 0 && (diff_yaw == 0)){
        std::cout<<"diff.norm() == 0 && (diff_yaw == 0)!\n";
        continue;
      }
      /* collision free */
      Eigen::Vector3d xt;
      isocc = false;
      double min_dist;
      int temp_check_num = check_num_;
      for(int k = 1; k <= temp_check_num; ++k){
        double tmparc = input[1] * double(k) / double(temp_check_num);
        Eigen::Vector2d tmpctrl;
        tmpctrl << input[0], tmparc;
        stateTransit(cur_state, tmpctrl, xt);
        if(fabs(tmparc) >= 1e-4){
          isocc = mm_config_->checkCarObsCollision(xt, false, false, min_dist);
        }
        if(isocc){break;}
      }
      if(isocc) continue;
      /* ---------- compute cost ---------- */
      double tmp_g_score = 0.0;
      if(singul>0) tmp_g_score += std::fabs(input[1]) * traj_forward_penalty_;
      else tmp_g_score += std::fabs(input[1]) * traj_back_penalty_;
      if(singul * cur_node->singul < 0) tmp_g_score += traj_gear_switch_penalty_;
      tmp_g_score += traj_steer_penalty_ * std::fabs(input[0]) * std::fabs(input[1]);
      tmp_g_score += traj_steer_change_penalty_ * std::fabs(input[0] - cur_node->input[0]);
      tmp_g_score += cur_node->g_score;
      double tmp_f_score = tmp_g_score + lambda_heu_ * getHeu(pro_state, end_state_.head(3));

      if(pro_node == NULL){
        pro_node = path_node_pool_[use_node_num_];
        pro_node->index = pro_id;
        pro_node->state = pro_state;
        pro_node->yaw_idx = pro_yaw_id;
        pro_node->f_score = tmp_f_score;
        pro_node->g_score = tmp_g_score;
        pro_node->input = input;
        pro_node->parent = cur_node;
        pro_node->node_state = IN_OPEN_SET;
        pro_node->singul = singul;
        open_set_.push(pro_node);

        expanded_nodes_.insert(pro_id, pro_yaw_id, pro_node);
        use_node_num_ += 1;
        if (use_node_num_ >= allocate_num_){
          std::cout << "[kino astar] run out of memory." << std::endl;
          return NO_PATH;
        }
      }else if (pro_node->node_state == IN_OPEN_SET){
        if (tmp_g_score < pro_node->g_score){
          pro_node->index = pro_id;
          pro_node->state = pro_state;
          pro_node->yaw_idx = pro_yaw_id;
          pro_node->f_score = tmp_f_score;
          pro_node->g_score = tmp_g_score;
          pro_node->input = input;
          pro_node->parent = cur_node;
          pro_node->singul = singul;
        }
      }else{
        std::cout << "[kino astar] error type in searching: " << pro_node->node_state << std::endl;
      }
    }
  }
  std::cout << "[kino astar] open set empty, no path." << std::endl;
  return NO_PATH;
}

int KinoAstar::yawToIndex(const double &yaw){
  double nor_yaw = normalize_angle(yaw);
  int idx = floor((nor_yaw - 0.0) * inv_yaw_resolution_);
  return idx;
}

double KinoAstar::normalize_angle(const double &angle){
  double nor_angle = angle;
  while (nor_angle > M_PI || nor_angle <= -M_PI){
    if(nor_angle > M_PI) nor_angle -= 2 * M_PI;
    if(nor_angle <= -M_PI) nor_angle += 2 * M_PI;
  }
  return nor_angle;
}

inline int KinoAstar::getSingularity(const double &vel){
  int singul = 0;
  if (fabs(vel) > non_siguav_){
    if (vel >= 0.0){singul = 1;} 
    else{singul = -1;}      
  }
  return singul;
}

inline double KinoAstar::getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2){
  return (x1 - x2).head(2).norm();
}

bool KinoAstar::is_shot_sucess(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2, int last_dir){
  std::vector<Eigen::Vector3d> path_list;
  double len;

  for(unsigned int i = 0; i < shotptr_list_.size(); i++){
    path_list.clear();
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ompl::base::StateSpacePtr shotptr = shotptr_list_[i];
    ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0]   = state2[0]; to[1]   = state2[1]; to[2]   = state2[2];
    std::vector<double> reals;
    len = shotptr->distance(from(), to());

    for (double l = 0.0; l <= len; l += oneshot_check_len_ / 2.0){
      shotptr->interpolate(from(), to(), l / len, s());
      reals = s.reals();
      path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
    }
    
    bool is_occ = false;
    double min_dist;
    int traj_pieces = 0;
    double traj_len = 0;
    bool is_forward = Eigen::Vector2d(cos(path_list[0](2)), sin(cos(path_list[0](2)))).transpose() * (path_list[1] - path_list[0]).head(2) > 0;
    int dir_last = is_forward ? 1 : -1;
    if(last_dir != dir_last) ++traj_pieces;
    ++traj_pieces;
    for(unsigned int j = 0; j < path_list.size(); ++j){
      if(j < path_list.size() - 1){
        is_forward = Eigen::Vector2d(cos(path_list[j](2)), sin(cos(path_list[j](2)))).transpose() * (path_list[j + 1] - path_list[j]).head(2) > 0;
        int dir_now = is_forward ? 1 : -1;
        if(dir_last != dir_now){
          traj_len = 0.0;
          ++traj_pieces;
          dir_last = dir_now;
          if(traj_pieces > 1){
            return false;
          }
        }else{
          traj_len += (path_list[j + 1] - path_list[j]).head(2).norm();
        }
      }
      if(mm_config_->checkCarObsCollision(path_list[j], false, false, min_dist)){
        is_occ = true;
        break;
      }
    }
    if(!is_occ){
      is_shot_succ_ = true;
      shotptrindex_ = i;
      return true;
    }  
  }
  
  return false;
}

double KinoAstar::computeShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2,const int shotptrind,
                                  std::vector<Eigen::Vector3d> &path_list, double& len){
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  ompl::base::StateSpacePtr shotptr = shotptr_list_[shotptrind];
  ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
  from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
  to[0]   = state2[0]; to[1]   = state2[1]; to[2]   = state2[2];
  std::vector<double> reals;
  len = shotptr->distance(from(), to());
  double sum_T = len / max_vel_;

  for (double l = 0.0; l <= len; l += oneshot_check_len_){
    shotptr->interpolate(from(), to(), l / len, s());
    reals = s.reals();
    path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
  }
  return sum_T;
}

void KinoAstar::retrievePath(const PathNodePtr &end_node){
  path_nodes_.clear();
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);
  while(cur_node->parent != NULL){
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

void KinoAstar::stateTransit(const Eigen::Vector3d &state0, const Eigen::Vector2d &ctrl_input, Eigen::Vector3d &state1){
  double psi = ctrl_input(0); 
  double s = ctrl_input(1);
  if(fabs(psi) > 1e-4){
    double k = mobile_base_wheel_base_ / tan(psi);
    state1(0) = state0(0) + k * (sin(state0(2) + s / k) - sin(state0(2)));
    state1(1) = state0(1) - k * (cos(state0(2) + s / k) - cos(state0(2)));
    state1(2) = state0(2) + s / k;
  }else{
    state1(0) = state0(0) + s * cos(state0(2));
    state1(1) = state0(1) + s * sin(state0(2));
    state1(2) = state0(2); 
  }
}

void KinoAstar::getKinoNode(){
  getSampleTrajs();
  if(has_path_) getTrajsWithTime();
}

void KinoAstar::simplifyRoute(const std::vector<Eigen::Vector3d> &node_path, 
                              const std::vector<Eigen::Vector2d> &node_input, 
                              std::vector<Eigen::Vector3d> &SampleList){
  if(node_path.size() == 2){
    SampleList.push_back(node_path[0]);
    SampleList.push_back(node_path[1]);
    return;
  }
  Eigen::Vector3d end_state = node_path.back();
  std::vector<Eigen::Vector3d> SampleList_temp;
  if(!is_shot_succ_){shotptrindex_ = 2;}
  for(unsigned int i = 0; i < node_path.size() - 1; ++i){ // from
    SampleList.push_back(node_path[i]);
    bool shot_succ = false;
    ompl::base::StateSpacePtr shotptr = shotptr_list_[shotptrindex_];
    ompl::base::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = node_path[i][0]; from[1] = node_path[i][1]; from[2] = node_path[i][2];
    unsigned int j;
    int piece_num_temp;
    double min_dist;
    for(j = node_path.size() - 1; j > i + 1; --j){
      if(i == 0) break;
      to[0] = node_path[j][0]; to[1] = node_path[j][1]; to[2] = node_path[j][2];
      double shotLength = shotptr->distance(from(), to());//oneshot的距离
      std::vector<double> reals;
      piece_num_temp = std::max(ceil(shotLength / oneshot_check_len_), 2.0);
      double check_len_temp = shotLength / (double)piece_num_temp;
      shot_succ = true;
      SampleList_temp.clear();
      for(double l = check_len_temp; l < shotLength - 1e-4; l += check_len_temp){
        shotptr->interpolate(from(), to(), l / shotLength, s());
        reals = s.reals();
        SampleList_temp.push_back(Eigen::Vector3d(reals[0], reals[1], normalize_angle(reals[2])));
        if(mm_config_->checkCarObsCollision(Eigen::Vector3d(reals[0], reals[1], normalize_angle(reals[2])), false, false, min_dist)){
          shot_succ = false;
          break;
        }
      }
      if(shot_succ){break;}
    }
    if(shot_succ){
      SampleList.insert(SampleList.end(), SampleList_temp.begin(), SampleList_temp.end());
      i = j - 1;
    }else{
      piece_num_temp = std::max(ceil(fabs(node_input[i + 1][1]) / oneshot_check_len_), 2.0);
      for(int k = 1; k < piece_num_temp; ++k){
        double tmparc = node_input[i + 1][1] * double(k) / double(piece_num_temp);
        Eigen::Vector2d tmpctrl;
        tmpctrl << node_input[i + 1][0], tmparc;
        Eigen::Vector3d state3d;
        stateTransit(node_path[i], tmpctrl, state3d);
        state3d[2] = normalize_angle(state3d[2]);
        SampleList.push_back(state3d);
      }
    }
  }
  SampleList.push_back(end_state);
  return;
}

void KinoAstar::getSampleTrajs(){
  std::vector<Eigen::Vector3d> roughSampleList;
  PathNodePtr node = path_nodes_.back();
  std::vector<Eigen::Vector3d> sample_path;
  while(node->parent != NULL){
    for (int k = check_num_; k > 0; --k){
      double tmparc = node->input[1] * double(k) / double(check_num_);
      Eigen::Vector2d tmpctrl;
      tmpctrl << node->input[0], tmparc;
      Eigen::Vector3d state3d;
      stateTransit(node->parent->state, tmpctrl, state3d);
      state3d[2] = normalize_angle(state3d[2]);
      roughSampleList.push_back(state3d);
    }
    sample_path.push_back(Eigen::Vector3d(node->state(0), node->state(1), 0.0));
    node = node->parent;
  }
  start_state_[2] = normalize_angle(start_state_[2]);
  roughSampleList.emplace_back(start_state_[0],start_state_[1],start_state_[2]);
  reverse(roughSampleList.begin(),roughSampleList.end());
  if(is_shot_succ_){
    ompl::base::StateSpacePtr shotptr = shotptr_list_[shotptrindex_];
    ompl::base::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    Eigen::Vector3d state1,state2;
    state1 = roughSampleList.back();
    state2 = end_state_.head(3);
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
    double shotLength = shotptr->distance(from(), to());
    shotptr->validSegmentCount(from(), to());
    std::vector<double> reals;
    std::vector<Eigen::Vector3d> roughSampleList_temp;
    Eigen::Vector3d state_now;
    roughSampleList_temp.clear();
    for(double l = oneshot_check_len_; l < shotLength; l += oneshot_check_len_){
      shotptr->interpolate(from(), to(), l / shotLength, s());
      reals = s.reals();
      state_now << reals[0], reals[1], normalize_angle(reals[2]);
      roughSampleList.emplace_back(state_now);
    }
    end_state_[2] = normalize_angle(end_state_[2]);
    roughSampleList.emplace_back(end_state_.head(3));
  }

  double tmp_len = 0;
  int truncate_idx = 0;
  for(truncate_idx = 0; truncate_idx < (int)roughSampleList.size() - 1; truncate_idx++){
    tmp_len += evaluateCurvatureWeightedDistance(roughSampleList[truncate_idx].head(2), roughSampleList[truncate_idx+1].head(2), roughSampleList[truncate_idx][2], roughSampleList[truncate_idx+1][2]);
  }
  roughSampleList.assign(roughSampleList.begin(), roughSampleList.begin() + truncate_idx + 1);
  sample_trajs_ = roughSampleList;
}

void KinoAstar::getTrajsWithTime(){
  double startvel = fabs(start_state_(3));
  double endvel = fabs(end_state_(3));
  std::vector<Eigen::Vector3d> traj_pts;
  std::vector<double> thetas;
  shot_lengthList.clear();
  shot_timeList.clear();
  shotindex.clear();
  shot_SList.clear();
  double tmpl = 0;
  int lastS = (sample_trajs_[1] - sample_trajs_[0]).head(2).dot(Eigen::Vector2d(cos(sample_trajs_[0][2]), sin(sample_trajs_[0][2]))) >= 0 ? 1 : -1;
  shotindex.push_back(0);

  for(unsigned int i = 0; i < sample_trajs_.size() - 1; i++){
    Eigen::Vector3d state1 = sample_trajs_[i];
    Eigen::Vector3d state2 = sample_trajs_[i + 1];
    int curS;
    if((state2 - state1).head(2).norm() < 1e-4){
      curS = lastS;
    }
    else{
      curS = (state2 - state1).head(2).dot(Eigen::Vector2d(cos(state1[2]), sin(state1[2]))) >=0 ? 1 : -1;
    }
    if(curS * lastS >= 0){
      tmpl += evaluateCurvatureWeightedDistance(state1.head(2), state2.head(2), state1[2], state2[2]); // change
    }else{
      shotindex.push_back(i);
      shot_SList.push_back(lastS);
      shot_lengthList.push_back(tmpl);
      if(tmpl == inf) return;
      shot_timeList.push_back(evaluateDuration(tmpl, non_siguav_, non_siguav_));
      tmpl = evaluateCurvatureWeightedDistance(state1.head(2), state2.head(2), state1[2], state2[2]);
    }
    lastS = curS;
  }
  shot_SList.push_back(lastS);
  shot_lengthList.push_back(tmpl);
  shot_timeList.push_back(evaluateDuration(tmpl, non_siguav_, non_siguav_));
  shotindex.push_back(sample_trajs_.size() - 1);

  if((int)shot_timeList.size() >= 2){
    shot_timeList[0] = evaluateDuration(shot_lengthList[0], startvel, non_siguav_);
    shot_timeList[shot_timeList.size() - 1] = evaluateDuration(shot_lengthList.back(), non_siguav_, endvel);
  }
  else{
    shot_timeList[0] = evaluateDuration(shot_lengthList[0], startvel, endvel);
  }
  // 
  for(unsigned int i = 0; i < shot_lengthList.size(); i++){
    double initv = non_siguav_, finv = non_siguav_;
    Eigen::Vector2d Initctrlinput, Finctrlinput;
    Initctrlinput<<0,0;Finctrlinput<<0,0;
    if(i == 0){
      initv = startvel; 
      Initctrlinput = start_ctrl_;
    }
    if(i == shot_lengthList.size() - 1)
      finv = endvel;

    double locallength = shot_lengthList[i];
    int sig = shot_SList[i];
    std::vector<Eigen::Vector3d> localTraj;
    localTraj.assign(sample_trajs_.begin() + shotindex[i], sample_trajs_.begin() + shotindex[i + 1] + 1);
    traj_pts.clear();
    thetas.clear();        
    double samplet;
    double tmparc = 0;
    int index = 0;
    double sample_time = sample_time_;
    if(shot_timeList[i] <= sample_time){
      sample_time = shot_timeList[i] / 2.0;
    }
    for(samplet = sample_time; samplet < shot_timeList[i]; samplet += sample_time){
      double arc = evaluateLength(samplet, locallength, shot_timeList[i], initv, finv);
      for(unsigned int k = index; k < localTraj.size() - 1; k++){
        tmparc += evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
        if(tmparc>=arc){
          index = k;
          double l1 = tmparc-arc;
          // double l = (localTraj[k+1]-localTraj[k]).head(2).norm();
          double l = evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
          
          double l2 = l-l1;//l2
          double px = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[0]; // FIXME l == 0
          double py = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[1];
          double yaw= (l1/l*localTraj[k]+l2/l*localTraj[k+1])[2];

          if(fabs(localTraj[k + 1][2] - localTraj[k][2]) >= M_PI){   
            double normalize_yaw = 0;
            if(localTraj[k + 1][2] <= 0){
              normalize_yaw = l1 / l * localTraj[k][2] + l2 / l * (localTraj[k + 1][2] + 2 * M_PI);
            }else if(localTraj[k][2] <=0){
              normalize_yaw = l1 / l * (localTraj[k][2] + 2 * M_PI) + l2 / l * localTraj[k + 1][2];
            }
            yaw = normalize_yaw;
          }

          traj_pts.emplace_back(px, py, sample_time);
          thetas.push_back(yaw);
          // tmparc -=(localTraj[k+1]-localTraj[k]).head(2).norm();
          tmparc -= evaluateCurvatureWeightedDistance(localTraj[k].head(2),localTraj[k+1].head(2),localTraj[k][2],localTraj[k+1][2]); //change
          break;
        }
      }
    }
    traj_pts.emplace_back(localTraj.back()[0], localTraj.back()[1], shot_timeList[i] - (samplet - sample_time_));
    thetas.push_back(localTraj.back()[2]);
    // flat_trajs_
    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    getFlatState(Eigen::Vector4d(localTraj.front()[0],localTraj.front()[1],localTraj.front()[2], initv), Initctrlinput, sig, startS);
    getFlatState(Eigen::Vector4d(localTraj.back()[0],localTraj.back()[1],localTraj.back()[2], finv), Finctrlinput, sig, endS);
    
    CarFlatTrajData flat_traj;
    flat_traj.traj_pts = traj_pts;
    flat_traj.thetas = thetas;
    flat_traj.start_state = startS;
    flat_traj.final_state = endS;
    flat_traj.singul = sig;
    flat_trajs_.push_back(flat_traj);
  }
  totalTrajTime = 0;
  for(unsigned int i = 0; i < shot_timeList.size(); i++){
    totalTrajTime += shot_timeList[i];
  }
}

double KinoAstar::evaluateCurvatureWeightedDistance(const Eigen::Vector2d &state1, const Eigen::Vector2d &state2, const double &yaw1, const double &yaw2){
  Eigen::Vector2d diff = state2 - state1;
  double EuDis = diff.norm();
  double siny = sin(yaw1), cosy = cos(yaw1);
  if( (Eigen::Vector2d(state1 + EuDis*Eigen::Vector2d(cos(yaw1),sin(yaw1)))-state2).norm()/EuDis < 0.01 ){
    return EuDis;
  }
  
  Eigen::Matrix2d A;
  Eigen::Vector2d b;

  A << cosy,siny,2*diff[0],2*diff[1];
  b << state1[0]*cosy+state1[1]*siny,diff.dot(state1+state2);
  b = A.colPivHouseholderQr().solve(b);
  double radius = (b-state1).norm();
  // std::cout<<"state1:"<<state1.transpose()<<"  "<<yaw1<<"  state2:"<<state2.transpose()<<"  "<<yaw2<<"   radius"<<radius<<std::endl;
  if(radius < min_turning_radius_){
    // std::cout<<"radius<min_turning_radius:"<<curvatureDisCoe_ * std::min(std::min(fabs(yaw2-yaw1),fabs(yaw2-yaw1+M_PI*2)),fabs(yaw2-yaw1-M_PI*2))<<std::endl;
    return curvatureDisCoe_ * std::min(std::min(fabs(yaw2-yaw1),fabs(yaw2-yaw1+M_PI*2)),fabs(yaw2-yaw1-M_PI*2));
  }
  // std::cout<<"noline EuDis:"<<EuDis<<std::endl;
  return EuDis;
}

double KinoAstar::evaluateDuration(const double &length, const double &startV, const double &endV){
  double critical_len; 
  if(startV > max_vel_ || endV > max_vel_){
    ROS_ERROR("kinoAstar:evaluateDuration:start or end vel is larger that the limit!");
  }
  double temp_max_acc = max_acc_ * 0.7;
  double temp_max_vel = max_vel_ * 1.0;
  double startv2 = pow(startV, 2);
  double endv2   = pow(endV, 2);
  double maxv2   = pow(temp_max_vel, 2);
  critical_len = (maxv2 - startv2) /(2 * temp_max_acc) + (maxv2 - endv2) / (2 * temp_max_acc);
  if(length >= critical_len){
    return (temp_max_vel - startV) / temp_max_acc + (temp_max_vel - endV) / temp_max_acc + (length - critical_len) / temp_max_vel;
  }
  else{
    double tmpv = sqrt(0.5 * (startv2 + endv2 + 2 * temp_max_acc * length));
    return (tmpv - startV) / temp_max_acc + (tmpv - endV) / temp_max_acc;
  }
}

double KinoAstar::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV){
  double critical_len; 
  if(startV > max_vel_ || endV > max_vel_){
    // std::cout << startV << ", " << endV << ", " << max_vel_ << std::endl;
    // ROS_ERROR("kinoAstar:evaluateLength:start or end vel is larger that the limit!");
  }
  double startv2 = pow(startV, 2);
  double endv2 = pow(endV, 2);
  double maxv2 = pow(max_vel_, 2);
  critical_len = (maxv2 - startv2) / (2 * max_acc_) + (maxv2 - endv2) / (2 * max_acc_);
  if(locallength >= critical_len){// 如果locallength距离比critical_len大，则为加速到到最大速度再减速
    double t1 = (max_vel_-startV)/max_acc_;
    double t2 = t1+(locallength-critical_len)/max_vel_;
    if(curt<=t1){
      return startV*curt + 0.5*max_acc_*pow(curt,2);
    }else if(curt<=t2){
      return startV*t1 + 0.5*max_acc_*pow(t1,2)+(curt-t1)*max_vel_;
    }else{
      return startV*t1 + 0.5*max_acc_*pow(t1,2) + (t2-t1)*max_vel_ + max_vel_*(curt-t2)-0.5*max_acc_*pow(curt-t2,2);
    }
  }else{
    double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc_*locallength));
    double tmpt = (tmpv-startV)/max_acc_;
    if(curt<=tmpt){
      return startV*curt+0.5*max_acc_*pow(curt,2);
    }
    else{
      return startV*tmpt+0.5*max_acc_*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*max_acc_*pow(curt-tmpt,2);
    }
  }
}

void KinoAstar::getFlatState(const Eigen::Vector4d &state, const Eigen::Vector2d &control_input,const int &singul, 
                             Eigen::MatrixXd &flat_state){

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);

    if (abs(vel) <= non_siguav_){
      vel = singul * non_siguav_;
    }
    else{
      vel = singul * vel;
    }
    flat_state << state.head(2), init_R * Eigen::Vector2d(vel, 0.0),
                  init_R * Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / mobile_base_wheel_base_ * std::pow(vel, 2));
}

Eigen::Vector3d KinoAstar::evaluatePos(const double &input_t){
  double t = input_t;
  if(t < 0 || t > totalTrajTime){
    // ROS_ERROR("In evaluatePos, t<0 || t>totalTrajTime");
    // std::cout << "t: " << t << ", totol: " << totalTrajTime << std::endl;
    t = std::min<double>(std::max<double>(0, t), totalTrajTime);
  }
  double startvel = fabs(start_state_(3));
  double endvel = fabs(end_state_(3));
  int index = -1;
  double tmpT = 0;
  double CutTime;
  for(unsigned int i = 0; i < shot_timeList.size(); i++){
    tmpT += shot_timeList[i];
    if(tmpT >= t) {
      index = i; 
      CutTime = t - tmpT + shot_timeList[i];
      break;
    }
  }

  double initv = non_siguav_, finv = non_siguav_;
  if(index==0) {initv = startvel;}
  if(index == (int)shot_lengthList.size() - 1) finv = endvel;
  double localtime = shot_timeList[index];
  double locallength = shot_lengthList[index];
  int front = shotindex[index];
  int back  = shotindex[index+1];
  std::vector<Eigen::Vector3d> localTraj;
  localTraj.assign(sample_trajs_.begin() + front, sample_trajs_.begin() + back + 1);
  double arclength = evaluateLength(CutTime, locallength, localtime, initv, finv);
  double tmparc = 0;
  for(unsigned int i = 0; i < localTraj.size() - 1; i++){
    // tmparc += (localTraj[i+1]-localTraj[i]).head(2).norm();// change
    tmparc += evaluateCurvatureWeightedDistance(localTraj[i].head(2), localTraj[i + 1].head(2), localTraj[i][2], localTraj[i + 1][2]); //change
    if(tmparc >= arclength){
      double l1 = tmparc - arclength;
      // double l = (localTraj[i+1]-localTraj[i]).head(2).norm();// change
      // FIXME l = 0
      double l = evaluateCurvatureWeightedDistance(localTraj[i].head(2), localTraj[i + 1].head(2), localTraj[i][2], localTraj[i + 1][2]); //change
      if(l < 1e-3) return localTraj[i];
      double l2 = l - l1;//l2
      Eigen::Vector3d state = l1 / l * localTraj[i] + l2 / l * localTraj[i + 1];
      if(fabs(localTraj[i + 1][2]-localTraj[i][2]) >= M_PI){   
        double normalize_yaw = 0;
        if(localTraj[i + 1][2] <= 0){
          normalize_yaw = l1 / l * localTraj[i][2] + l2 / l * (localTraj[i + 1][2] + 2 * M_PI);
        }else if(localTraj[i][2] <= 0){
          normalize_yaw = l1 / l * (localTraj[i][2] + 2 * M_PI) + l2 / l * localTraj[i + 1][2];
        }
        state[2] = normalize_yaw;
      }
      return state;
    }
  }
  return localTraj.back();
}

} // namespace remani_planner
