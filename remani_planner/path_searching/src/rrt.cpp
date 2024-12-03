#include "path_searching/rrt.h"

namespace remani_planner{

  int RrtPlanning::RRTSearchAndGetSimplePath(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list, 
                                              const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
                                              const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list, const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
                                              const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list,
                                              std::vector<Eigen::VectorXd>& path, std::vector<double>& yaw_list, std::vector<double>& t_list)
  {
    std::vector<Eigen::VectorXd> path_full;
    std::vector<double> t_list_full, yaw_list_full;
    if(start_pt_list.size() < 1){
      ROS_ERROR("[RRT plan]: start pt < 1");
      return false;
    }
    if(end_pt_list.size() < 1){
      ROS_ERROR("[RRT plan]: end pt < 1");
      return false;
    }
    
    init(start_pt_list, end_pt_list);
    bool status = search(start_pt_list, start_yaw_list, end_pt_list, end_yaw_list, start_g_score_list, start_layer_list, end_g_score_list, end_layer_list, start_singul_list, end_singul_list);

    if (status == false){
      return status;
      // cout << "[RRT replan]: RRT search fail!" << endl;
    }else{
      // cout << "[RRT replan]: RRT search success." << endl;
    }
    getTraj(path_full, yaw_list_full, t_list_full);

    path.clear();
    t_list.clear();
    yaw_list.clear();
    Eigen::VectorXd temp(traj_dim_);
    temp = path_full[0];

    path.push_back(temp);
    yaw_list.push_back(yaw_list_full[0]);

    double t_total = 0.0;
    for(size_t i = 0; i < t_list_full.size(); i++){
      t_total += t_list_full[i];
      temp = path_full[i + 1];
      path.push_back(temp);
      yaw_list.push_back(yaw_list_full[i + 1]);
      t_list.push_back(t_total);
      t_total = 0.0;
    }

    return status;
  }

  bool RrtPlanning::search(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list, 
                          const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
                          const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list, const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
                          const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list){
    // std::cout << "[sample mani]: Search begin. start: " << start_state.transpose() << " end: " << end_state.transpose() << std::endl;
    ros::Time time_1 = ros::Time::now();
    PathNodeRRTPtr start_node, end_node;
    PathNodeRRTPtr path_node_1 = nullptr, path_node_2 = nullptr;

    int start_num = start_pt_list.size();
    if(start_num < 1){
      ROS_ERROR("[RRT]: Fail! Start list size zero!");
      return false;
    }
    for(int i = 0; i < start_num; ++i){
      start_node = new PathNodeRRT(traj_dim_);
      start_node->state = start_pt_list[i];
      start_node->yaw = start_yaw_list[i];
      start_node->node_state = PathNodeRRT::NODE_STATE::IN_TREE;
      start_node->g_score = start_g_score_list[i];
      start_node->index = calculateValue(start_node);
      start_node->layer = start_layer_list[i];
      start_node->singul = start_singul_list[i];
      // std::cout << "test: " << node_pool_.begin()->second->index << std::endl;
      node_pool_.insert(std::make_pair(start_node->index, start_node));
      // std::cout << "start: " << start_node->state.transpose() << std::endl;
      ++tree_count_;
    }

    int end_num = end_pt_list.size();
    if(end_num < 1){
      ROS_ERROR("[RRT]: Fail! End list size zero!");
      return false;
    }
    for(int i = 0; i < end_num; ++i){
      end_node = new PathNodeRRT(traj_dim_);
      end_node->state = end_pt_list[i];
      end_node->yaw = end_yaw_list[i];
      end_node->node_state = PathNodeRRT::NODE_STATE::IN_ANTI_TREE;
      end_node->g_score = end_g_score_list[i];
      end_node->index = calculateValue(end_node);
      end_node->layer = end_layer_list[i];
      end_node->singul = end_singul_list[i];
      // std::cout << "test: " << node_pool_.begin()->second->index << std::endl;
      node_pool_.insert(std::make_pair(end_node->index, end_node));
      // std::cout << "end: " << end_node->state.transpose() << std::endl;
      ++anti_tree_count_;
    }

    // std::cout << "start: " << start_num << " end: " << end_num << std::endl;
    // std::cout << node_pool_.size() << std::endl;

    int loop = 0;
    PathNodeRRTPtr q_new, q_near;
    PathNodeRRTPtr q_new_1, q_near_1;
    PathNodeRRTPtr q_new_2;
    Eigen::VectorXd rand_s;
    double rand_yaw;
    bool dir = false;
    // Eigen::VectorXd s_rand;
    // std::cout << "end: " << end_state.transpose() << " " << end_yaw << std::endl;
    while(true){
      // std::cout << loop << std::endl;
      // std::cout << loop << std::endl;
      if( (have_path_ && (ros::Time::now() - time_1).toSec() > 0.01) || (ros::Time::now() - time_1).toSec() > max_sample_time_){
        break;
      }
      if(tree_count_ > anti_tree_count_){
        dir = false;
      }
      else{
        dir = true;
      }
      ++loop;
      // if(random_dis_(random_gen_) < goal_rate_){
      //   rand_s = end_state;
      //   rand_yaw = end_yaw;
      // }
      // else{
      sample(rand_s, rand_yaw);
      // }

      // std::cout << "q_rand: " << rand_s.transpose() << " " << rand_yaw << std::endl;
      q_near = near(rand_s, rand_yaw, dir);
      
      if(q_near == nullptr){
        std::cout << "test 1" << std::endl;
        continue;
      }
      // std::cout << "q_near: " << q_near->state.transpose() << " " << q_near->yaw << std::endl;

      // q_new = steer(q_near, s_rand, 0.4 + 0.3 * random_dis_(random_gen_));
      q_new = steer(q_near, rand_s, rand_yaw, time_resolution_);
      if(q_new == nullptr){
        // std::cout << "test 2" << std::endl;
        continue;
      }
      // std::cout << "rand: " << rand_s.transpose() << std::endl;
      // std::cout << "near: " << q_near->state.transpose() << std::endl;
      // std::cout << "new:  " << q_new->state.transpose() << std::endl;
      // std::cout << "test 4" << std::endl;
      if((dir && q_new->node_state == PathNodeRRT::IN_ANTI_TREE) || (!dir && q_new->node_state == PathNodeRRT::IN_TREE)){
        // ROS_WARN("[RRT Plan]: Success. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
        have_path_ = true;
              // std::cout << "11111111111$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
              // std::cout << "cost: " << q_near->g_score + q_new->g_score + estimateHeuristic(q_near, q_new) << std::endl;
        if(q_near->g_score + q_new->g_score + estimateHeuristic(q_near, q_new) < c_max_){
          // std::cout << "11111111" << std::endl;
          c_max_ = q_near->g_score + q_new->g_score + estimateHeuristic(q_near, q_new);
          // std::cout << "c_max_: " << c_max_ << std::endl;
          path_node_1 = q_near;
          path_node_2 = q_new;
        }
        continue;
        // mergeTree(q_near, q_new);
        // return true;
      }

      if(q_new->node_state == PathNodeRRT::EXPAND || (q_new->node_state == q_near->node_state && q_new->g_score > q_near->g_score + estimateHeuristic(q_near, q_new))){
        linkNode(q_near, q_new);
        q_new->node_state = q_near->node_state;
        // q_new->g_score = q_near->g_score + estimateHeuristic(q_near, q_new);
        if(q_new->node_state == PathNodeRRT::IN_TREE)
          ++tree_count_;
        else 
          ++anti_tree_count_;
        rewire(q_new, 0.45);

        q_near_1 = near(q_new->state, q_new->yaw, !dir);
        if(q_near_1 == nullptr){
          std::cout << "test 3" << std::endl;

          continue;
        }

        q_new_1 = steer(q_near_1, q_new->state, q_new->yaw, time_resolution_);
        if(q_new_1 == nullptr){
          // std::cout << "test 4" << std::endl;

          continue;
        }

        if((!dir && q_new_1->node_state == PathNodeRRT::IN_ANTI_TREE) || (dir && q_new_1->node_state == PathNodeRRT::IN_TREE)){
          // ROS_WARN("[RRT Plan]: Success. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
          have_path_ = true;
          // std::cout << "222222222222$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
          // std::cout << "cost: " << q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1) << std::endl;
          // std::cout << "21" << std::endl;
          if(q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1) < c_max_){
            // std::cout << "222222222222222" << std::endl;
            c_max_ = q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1);
            // std::cout << "c_max_: " << c_max_ << std::endl;
            path_node_1 = q_new_1;
            path_node_2 = q_near_1;
          }
          continue;
          // mergeTree(q_new_1, q_near_1);
          // return true;
        }
          // std::cout << "q_new: " << q_new->node_state << std::endl;
          // std::cout << "q_near_1: " << q_near_1->node_state << std::endl;

        if(q_new_1->node_state == PathNodeRRT::EXPAND || (q_new_1->node_state == q_near_1->node_state && q_new_1->g_score > q_near_1->g_score + estimateHeuristic(q_near_1, q_new_1))){
          linkNode(q_near_1, q_new_1);
          q_new_1->node_state = q_near_1->node_state;
          // q_new_1->g_score = q_near_1->g_score + estimateHeuristic(q_near_1, q_new_1);
          rewire(q_new_1, 0.45);
          if(q_new_1->node_state == PathNodeRRT::IN_TREE)
            ++tree_count_;
          else 
            ++anti_tree_count_;

          // std::cout << "q_new: " << q_new->node_state << std::endl;
          // std::cout << "q_near_1: " << q_near_1->node_state << std::endl;
          // std::cout << "q_new_1: " << q_new_1->node_state << std::endl;
          // int temp_test = 0;
          while( !((q_new->state - q_new_1->state).norm() < 1.0e-2 && fabs(q_new->yaw - q_new_1->yaw) < 1.0e-2) ){
            q_new_2 = steer(q_new_1, q_new->state, q_new->yaw, time_resolution_);
            // std::cout << "q_new_1: " << q_new_1->state.transpose() << " " << q_new_1->yaw << std::endl;
            // std::cout << "q_new: " << q_new->state.transpose() << " " << q_new->yaw << std::endl;
            // if(q_new_2 == nullptr){
            //   std::cout << "nullptr!!!!!!!!!!!!!!!" << std::endl;
            //   break;
            //   }
            // std::cout << "q_new_2: " << q_new_2->state.transpose() << " " << q_new_2->yaw << std::endl;
            if(q_new_2 != nullptr && (q_new_2->node_state == PathNodeRRT::EXPAND || (q_new_2->node_state == q_new_1->node_state && q_new_2->g_score > q_new_1->g_score + estimateHeuristic(q_new_1, q_new_2)))){
              // std::cout << "1" << std::endl;
              linkNode(q_new_1, q_new_2);
              q_new_2->node_state = q_new_1->node_state;
              // q_new_2->g_score = q_new_1->g_score + estimateHeuristic(q_new_1, q_new_2);
              rewire(q_new_2, 0.45);
              if(q_new_2->node_state == PathNodeRRT::IN_TREE)
                ++tree_count_;
              else 
                ++anti_tree_count_;
              q_new_1 = q_new_2;
            }
            else if(q_new_2 != nullptr && ((dir && q_new_2->node_state == PathNodeRRT::IN_TREE) || (!dir && q_new_2->node_state == PathNodeRRT::IN_ANTI_TREE))){
            // ROS_WARN("[RRT Plan]: Success. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
              // std::cout << "3333333$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
              // std::cout << "cost: " << q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1) << std::endl;
              // std::cout << "22" << std::endl;

              have_path_ = true;
              if(q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1) < c_max_){
                // std::cout << "3333333333333" << std::endl;
                c_max_ = q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1);
                // std::cout << "c_max_: " << c_max_ << std::endl;
                path_node_1 = q_new_2;
                path_node_2 = q_new_1;
              }
              // mergeTree(q_new_2, q_new_1);
              // return true;
              break;
            }
            else if(q_new_2 != nullptr && q_new_2->node_state == q_new_1->node_state && q_new_2->g_score < q_new_1->g_score + estimateHeuristic(q_new_1, q_new_2)){
              // std::cout << "333" << std::endl;

              q_new_1 = q_new_2;
            }
            else{
              // std::cout << "4444" << std::endl;

              break;
            }
          }

        }
      }
    }
    int count_1_posi = 0, count_1_negi = 0, count_2_posi = 0, count_2_negi = 0;
    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      if(it->second->children.size() == 0 && it->second->node_state == PathNodeRRT::IN_TREE && it->second->singul == 1){
        count_1_posi++;
      } 
      else if(it->second->children.size() == 0 && it->second->node_state == PathNodeRRT::IN_TREE && it->second->singul == -1){
        count_1_negi++;
      }
      else if(it->second->children.size() == 0 && it->second->node_state == PathNodeRRT::IN_ANTI_TREE && it->second->singul == 1){
        count_2_posi++;
      }
      else if(it->second->children.size() == 0 && it->second->node_state == PathNodeRRT::IN_ANTI_TREE && it->second->singul == -1){
        count_2_negi++;
      } 
    }
    // std::cout << "tree 1: " << count_1_posi << " " << count_1_negi << std::endl;
    // std::cout << "tree 2: " << count_2_posi << " " << count_2_negi << std::endl;


    if(!have_path_){
      // ROS_ERROR("[RRT Plan]: Fail. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, (int)node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
      return false;
    }else{
      // std::cout << "last cost: " << path_node_1->g_score + path_node_2->g_score + estimateHeuristic(path_node_1, path_node_2) << std::endl;
      // ROS_INFO("[RRT Plan]: Success. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, (int)node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
      // std::cout << "debug 1\n";
      mergeTree(path_node_1, path_node_2);
      // std::cout << "debug 2\n";
      return true;
    }
  }

  PathNodeRRTPtr RrtPlanning::initNode(const Eigen::VectorXd &s, const double yaw){
    auto it = node_pool_.find(calculateValue(s, yaw));
    if(it != node_pool_.end()){
      return it->second;
    }
    PathNodeRRTPtr node = new PathNodeRRT;
    node->node_state = PathNodeRRT::NODE_STATE::EXPAND;
    node->state = s;
    node->yaw = yaw;
    // node->f_score = estimateHeuristic(node, this->end_node_);
    node->index = calculateValue(node);
    node_pool_.insert(std::make_pair(node->index, node));
    return node;
  }

  void RrtPlanning::sample(Eigen::VectorXd &s_state, double &s_yaw){
    Eigen::VectorXd sample_state(traj_dim_);
    // for(int i = 0; i < traj_dim_; ++i){
    //   sample_state(i) = min_size_[i] + (max_size_[i] - min_size_[i]) * random_dis_(random_gen_);
    // }
    // s_state = sample_state;
    // s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);
    if(c_max_ < 1.0e5){
      Eigen::VectorXd s_center = (sample_start_ + sample_end_) / 2.0;
      Eigen::VectorXd a1 = (sample_end_ - sample_start_).normalized();
      Eigen::MatrixXd M = a1 * Eigen::VectorXd::Ones(traj_dim_).transpose();
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::MatrixXd V = svd.matrixV();
      Eigen::MatrixXd U = svd.matrixU();
      Eigen::MatrixXd C_middle(traj_dim_, traj_dim_);
      C_middle.setZero();
      for(int i = 0; i < traj_dim_ - 1; ++i)
        C_middle(i, i) = 1.0;
      C_middle(traj_dim_ - 1, traj_dim_ - 1) = U.determinant() * V.determinant();

      Eigen::MatrixXd C = U * C_middle;
      Eigen::MatrixXd L(traj_dim_, traj_dim_);
      L.setZero();
      double c_max = c_max_ + (sample_start_radius_ + sample_end_radius_) / 2.0;
      for(int i = 1; i < traj_dim_; ++i)
        L(i, i) = sqrt(c_max * c_max - c_min_ * c_min_) / 2;
      L(0, 0) = c_max / 2.0;
      Eigen::VectorXd phi(traj_dim_ - 1);
      double r;

      // Uniform sampling within unit sphere
      for(int i = 0; i < traj_dim_; ++i){
        sample_state(i) = norm_dis_(random_gen_);
      }
      sample_state.normalize();
      // r = cbrt(random_dis_(random_gen_));
      r = pow(random_dis_(random_gen_), 1.0/traj_dim_);
      sample_state *= r;

      s_state = C * L * sample_state + s_center;
      s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);
      
      for(int i = 0; i < traj_dim_; ++i){
        s_state[i] = min(max(min_size_[i], s_state[i]), max_size_[i]);
      }
    }
    else{
      for(int i = 0; i < traj_dim_; ++i){
        sample_state(i) = min_size_[i] + (max_size_[i] - min_size_[i]) * random_dis_(random_gen_);
      }
      s_state = sample_state;
      s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);
    }
  }

  PathNodeRRTPtr RrtPlanning::near(const Eigen::VectorXd &s, const double yaw, bool flag){
    PathNodeRRTPtr q_near = nullptr;
    PathNodeRRTPtr node;
    double min_dis = 1.0e6;
    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      node = it->second;
      if((flag && node->node_state == PathNodeRRT::NODE_STATE::IN_TREE) || (!flag && node->node_state == PathNodeRRT::NODE_STATE::IN_ANTI_TREE)){
        if(((fabs(yaw - node->yaw) > 1.0e-2) || ((s - node->state).norm() > 1.0e-2)) && estimateHeuristic(s, yaw, node->state, node->yaw) < min_dis){
          min_dis = estimateHeuristic(s, yaw, node->state, node->yaw);
          q_near = node;
        }
      }
    }
    return q_near;
  }

  PathNodeRRTPtr RrtPlanning::steer(PathNodeRRTPtr &q_near, const Eigen::VectorXd &s_rand, const double s_yaw, double step_time){
    Eigen::VectorXd dir = s_rand - q_near->state;
    Eigen::VectorXd s_new(traj_dim_);
    // std::cout << "max_vel: " << max_vel_ << std::endl;
    //     std::cout << "max_joint_vel_: " << max_joint_vel_ << std::endl;
    int singul;
    if(q_near->parent == nullptr){
      singul = (((s_rand - q_near->state).head(2)).dot(Eigen::Vector2d(cos(q_near->yaw), sin(q_near->yaw)))) >= 0 ? 1:-1;
      // if(q_near->singul != 0 && singul != q_near->singul)
      //   return nullptr;
    }
    else{
      singul = (((s_rand - q_near->state).head(2)).dot((q_near->state - q_near->parent->state).head(2))) >= 0 ? q_near->singul : -q_near->singul;
    }
    if(q_near->singul != 0 && singul != q_near->singul)
      singul = q_near->singul;



    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    from[0] = q_near->state[0]; from[1] = q_near->state[1]; 
    if(singul == 1)
      from[2] = q_near->yaw;
    else{
      from[2] = q_near->yaw + M_PI;
      if(from[2] > M_PI)
        from[2] -= 2 * M_PI;
    }

    to[0]   = s_rand[0]; to[1]   = s_rand[1]; 
    if(singul == 1)
      to[2] = s_yaw;
    else{
      to[2] = s_yaw + M_PI;
      if(to[2] > M_PI)
        to[2] -= 2 * M_PI;
    }

    double len = dubins_curve_->distance(from(), to());
    // std::cout << "len: " << len << std::endl;
    double yaw;
    if(len > max_vel_ * step_time){
      dubins_curve_->interpolate(from(), to(), max_vel_ * step_time / len, s());
      auto reals = s.reals();
      s_new(0) = reals[0]; 
      s_new(1) = reals[1];
      yaw = reals[2];
      // std::cout << "real: " << reals[2] << std::endl;
      // std::cout << "to: " << to[2] << std::endl;
    }
    else{
      s_new.head(mobile_base_dof_) = s_rand.head(mobile_base_dof_);
      yaw = to[2];
    }

    if(singul == -1){
      yaw += M_PI;
      if(yaw > M_PI)
        yaw -= 2 * M_PI;
    }

    if(dir.tail(manipulator_dof_).norm() > 1.0e-2){
      s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_) + dir.tail(manipulator_dof_).normalized() * max_joint_vel_ * step_time;
    }
    else{
      s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_);
    }

    for(int i = mobile_base_dof_; i < traj_dim_; ++i){
      if(s_new(i) < std::min(s_rand(i), q_near->state(i)) || s_new(i) > std::max(s_rand(i), q_near->state(i))){
        s_new(i) = s_rand(i);
      }
    }
    // if(yaw < std::min(s_yaw, q_near->yaw) || yaw > std::max(s_yaw, q_near->yaw)){
    //   yaw = s_yaw;
    // }

    // if((fabs(yaw - q_near->yaw) > M_PI ? (2.0 * M_PI - fabs(yaw - q_near->yaw)) : (fabs(yaw - q_near->yaw))) > M_PI / 6.0){
    //   return nullptr;
    // }
    
    // Eigen::Vector2d straite;
    // if(((s_new - q_near->state).head(2)).norm() > 1.0e-3){
    //   straite = ((s_new - q_near->state).head(2)).normalized();
    //   if(fabs(straite.dot(Eigen::Vector2d(cos(q_near->yaw), sin(q_near->yaw)))) < cos(M_PI / 6)){
    //     return nullptr;
    //   }
    // }

    if(checkcollision(q_near, s_new, yaw)){
      return nullptr;
    }

    PathNodeRRTPtr q_new = initNode(s_new, yaw);
    return q_new;
  }

  void RrtPlanning::rewire(PathNodeRRTPtr q_new, double near_time){
    std::vector<PathNodeRRTPtr> neighbour;
    PathNodeRRTPtr temp;
    bool flag;
    int num = node_pool_.size();
    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      temp = it->second;
      flag = false;
      if(temp->node_state != q_new->node_state){
        continue;
      }
      if(temp == q_new){
        continue;
      }
      if((temp->state.head(mobile_base_dof_) - q_new->state.head(mobile_base_dof_)).norm() > max_vel_ * near_time)
        continue;
      for(int j = 0; j < manipulator_dof_; ++j){
        if(calAngleErr(temp->state(mobile_base_dof_ + j), q_new->state(mobile_base_dof_ + j)) > max_joint_vel_ * near_time){
          flag = true;
          break;
        }
      }
      if(flag)
        continue;
      neighbour.push_back(temp);
    }

    num = neighbour.size();
    if(num < 1)
      return;

    temp = nullptr;
    double min = q_new->g_score;
    for(int i = 0; i < num; ++i){
      if(neighbour[i]->g_score + estimateHeuristic(neighbour[i], q_new) < min && !checkcollision(neighbour[i], q_new)){
        min = neighbour[i]->g_score + estimateHeuristic(neighbour[i], q_new);
        temp = neighbour[i];
      }
    }
    
    if(temp != nullptr)
    {
      // std::cout << "rewire!" << std::endl;
      linkNode(temp, q_new);
    }

    for(int i = 0; i < num; ++i){
      if(q_new->g_score + estimateHeuristic(q_new, neighbour[i]) < neighbour[i]->g_score && !checkcollision(neighbour[i], q_new)){
        // std::cout << "rewire!!" << std::endl;
        linkNode(q_new, neighbour[i]);
        // neighbour[i]->g_score = q_new->g_score + estimateHeuristic(q_new, neighbour[i]);
      }
    }
  }

  void RrtPlanning::linkNode(PathNodeRRTPtr &parent, PathNodeRRTPtr &child){
    // assert(parent == child);
    PathNodeRRTPtr pre_parent = child->parent;
    if(pre_parent == parent){
      return;
    }
    else if(pre_parent != nullptr){
      pre_parent->children.erase(child->index);
    }
    child->parent = parent;    
    parent->children.insert(std::make_pair(child->index, child));
    int singul;
    if(parent->parent != nullptr){
      singul = ((child->state - parent->state).head(2)).dot((parent->state - parent->parent->state).head(2)) >= 0 ? parent->singul : -parent->singul;

    }
    else{
      singul = ((child->state - parent->state).head(2)).dot(Eigen::Vector2d(cos(parent->yaw), sin(parent->yaw))) >= 0 ? 1 : -1;
    }
    
    child->singul = singul;
    if(parent->singul != 0 && singul != parent->singul){
      child->singul = parent->singul;
    }
    expandGscore(child);
  }

  void RrtPlanning::expandGscore(PathNodeRRTPtr q){
    if(q->g_score == q->parent->g_score + estimateHeuristic(q->parent, q)){
      return;
    }
    q->g_score = q->parent->g_score + estimateHeuristic(q->parent, q);
    for(auto it = q->children.begin(); it != q->children.end(); ++it){
      expandGscore(it->second);
    }
  }

  double RrtPlanning::estimateHeuristic(PathNodeRRTPtr &x1, PathNodeRRTPtr &x2){ 
    double ret = estimateHeuristic(x1->state, x1->yaw, x2->state, x2->yaw);
    // int singul;
    // if(x1->parent != nullptr){
    //   singul = ((x2->state - x1->state).head(2)).dot((x1->state - x1->parent->state).head(2)) >= 0 ? x1->singul : -x1->singul;

    // }
    // else{
    //   singul = ((x2->state - x1->state).head(2)).dot(Eigen::Vector2d(cos(x1->yaw), sin(x1->yaw))) >= 0 ? 1 : -1;
    // }
    // if(singul != x1->singul){
    //   ret *= 3.0;
    // }
    // if(singul == -1){
    //   ret *= 1.2;
    // }
    return ret;
  }

  void RrtPlanning::mergeTree(PathNodeRRTPtr &s1, PathNodeRRTPtr &s2){
    

    PathNodeRRTPtr q1, q2, q_temp;
    if(s1->node_state == PathNodeRRT::IN_TREE){
      q1 = s1;
      q2 = s2;
    }
    else{
      q1 = s2;
      q2 = s1;
    }


    std::vector<PathNodeRRTPtr> s_list;
    q_temp = q2;
    while(q_temp != nullptr){
      s_list.push_back(q_temp);
      q_temp->node_state = PathNodeRRT::IN_TREE;
      q_temp->children.clear();
      q_temp = q_temp->parent;
    }
        // cout << "test 3" << std::endl;
    linkNode(q1, s_list[0]);
    // std::cout << "size: " << s_list.size() << std::endl;
    for(unsigned int i = 1; i < s_list.size(); ++i){
      // s_list[i]->parent = s_list[i-1];
      linkNode(s_list[i-1], s_list[i]);
    }
    // s_list[0]->parent= q1;
    this->end_node_ = s_list.back();
  }

  double RrtPlanning::estimateHeuristic(const Eigen::VectorXd& s1, const double& yaw1, const Eigen::VectorXd& s2, const double& yaw2){
    double ret = 0.0;
    // ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_);
    // from[0] = s1[0]; from[1] = s1[1]; from[2] = yaw1;
    // to[0]   = s2[0]; to[1]   = s2[1]; to[2]   = yaw2;

    // ret += dubins_curve_->distance(from(), to());

    // for(int i = 0; i < manipulator_dof_; ++i){
    //   ret += calAngleErr(s1(mobile_base_dof_ + i), s2(mobile_base_dof_ + i));
    // }
    ret += sqrt((s1 - s2).squaredNorm() + (yaw1 - yaw2) * (yaw1 - yaw2));
    // ret = (s1-s2).norm();
    return ret;
  }

  string RrtPlanning::calculateValue(PathNodeRRTPtr &q){
    return calculateValue(q->state, q->yaw);
  }
  
  string RrtPlanning::calculateValue(const Eigen::VectorXd &s, const double yaw){
    int k;
    string ret;
    for(int i = 0; i < traj_dim_; ++i){
      k = round(s(i) * 100.0);
      string s(std::to_string(k));
      ret += s;
    }
    ret += std::to_string(round(yaw * 100.0));
    return ret;
  }
  
  // double RrtPlanning::estimateHeuristic(const Eigen::VectorXd &s1, const Eigen::VectorXd &s2){
  //   double ret = 0.0;
  //   ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_);
  //   from[0] = s1[0]; from[1] = s1[1]; from[2] = s1[2];
  //   to[0]   = s2[0]; to[1]   = s2[1]; to[2]   = s2[2];

  //   ret += dubins_curve_->distance(from(), to());

  //   for(int i = 0; i < manipulator_dof_; ++i){
  //     ret += calAngleErr(s1(mobile_base_dof_ + i), s2(mobile_base_dof_ + i));
  //   }
  //   return ret;
  // }

  void RrtPlanning::init(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<Eigen::VectorXd>& end_pt_list){
    this->reset();
    int start_num = start_pt_list.size();
    int end_num = end_pt_list.size();
    for(int i = 0; i < mobile_base_dof_; ++i){
      max_size_[i] = -1.0e6;
      min_size_[i] = 1.0e6;
    }
    c_max_ = 1.0e6;
    sample_start_radius_ = 0.0;
    if(start_num == 1){
      sample_start_ = start_pt_list[0];
      sample_start_radius_ = 0;
    }
    else{
      for(int i = 0; i < start_num; ++i){
        for(int j = i + 1; j < start_num; ++j){
          if((start_pt_list[i] - start_pt_list[j]).norm() > sample_start_radius_){
            sample_start_ = (start_pt_list[i] + start_pt_list[j]) / 2.0;
            sample_start_radius_ = (start_pt_list[i] - start_pt_list[j]).norm();
          }
        }
      }
    }

    sample_end_radius_ = 0.0;
    if(end_num == 1){
      sample_end_ = end_pt_list[0];
      sample_end_radius_ = 0;
    }
    else{
      for(int i = 0; i < end_num; ++i){
        for(int j = i + 1; j < end_num; ++j){
          if((end_pt_list[i] - end_pt_list[j]).norm() > sample_end_radius_){
            sample_end_ = (end_pt_list[i] + end_pt_list[j]) / 2.0;
            sample_end_radius_ = (end_pt_list[i] - end_pt_list[j]).norm();
          }
        }
      }
    }

    c_min_ = (sample_start_ - sample_end_).norm();

    // for(int i = 0; i < start_num; ++i){
    //   std::cout << "start: " << start_pt_list[i].transpose() << std::endl;
    // }
    // std::cout << "sample start: " << sample_start_.transpose() << std::endl;
    // std::cout << "start radius: " << sample_start_radius_ << std::endl;

    // for(int i = 0; i < end_num; ++i){
    //   std::cout << "end: " << end_pt_list[i].transpose() << std::endl;
    // }
    // std::cout << "sample end: " << sample_end_.transpose() << std::endl;
    // std::cout << "end radius: " << sample_end_radius_ << std::endl;
    // std::cout << "c min: " << c_min_ << std::endl;

    for(int i = 0; i < start_num; ++i)
      for(int j = 0; j < end_num; ++j)
        for(int k = 0; k < mobile_base_dof_; ++k){
          max_size_[k] = max(start_pt_list[i][k] + fabs(end_pt_list[j][k] - start_pt_list[i][k]) + 20.0, max_size_[k]);
          min_size_[k] = min(start_pt_list[i][k] - fabs(end_pt_list[j][k] - start_pt_list[i][k]) - 20.0, min_size_[k]);
        }
  }

  double RrtPlanning::calAngleErr(double angle1, double angle2){
    return fabs(angle1 - angle2);
  }

  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state){
    Eigen::Vector3d xt;
    xt[0] = cur_state->state[0];
    xt[1] = cur_state->state[1];
    xt[2] = cur_state->yaw; 
    return mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_), false);
  }

  // bool RrtPlanning::checkcollision(Eigen::VectorXd cur_state){
  //   return mm_config_->checkManicollision(cur_state.head(mobile_base_dof_ + 1), cur_state.tail(manipulator_dof_), false);
  // }

  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state, const Eigen::VectorXd& next_state, const double next_yaw){
    
    if(cur_state->node_state == PathNodeRRT::NODE_STATE::COLLISION){
      return true;
    }

    //check feasibility
    // if((cur_state->state - next_state).head(mobile_base_dof_).norm() < 1.0e-3)
    //   return true;
    // Eigen::Vector2d cur_yaw_dir(cos(cur_state->yaw), sin(cur_state->yaw));
    // Eigen::Vector2d next_yaw_dir(cos(next_yaw), sin(next_yaw));
    // Eigen::Vector2d state_dir = ((next_state - cur_state->state).head(mobile_base_dof_)).normalized();
    // if((cur_yaw_dir.dot(state_dir) * next_yaw_dir.dot(state_dir)) < 0)
    //   return true;

    int singul;
    if(cur_state->parent == nullptr){
      singul = ((next_state - cur_state->state).head(2)).dot(Eigen::Vector2d(cos(cur_state->yaw), sin(cur_state->yaw))) >= 0 ? 1:-1;
      // if(cur_state->singul != 0 && singul != cur_state->singul)
      //   return true;
    }
    else
      singul = ((next_state - cur_state->state).head(2)).dot((cur_state->state - cur_state->parent->state).head(2)) >= 0 ? cur_state->singul : -cur_state->singul;    
    if(cur_state->singul != 0 && singul != cur_state->singul){
      singul = cur_state->singul;
    }

    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    from[0] = cur_state->state[0]; from[1] = cur_state->state[1]; from[2] = cur_state->yaw;
    to[0]   = next_state[0]; to[1]   = next_state[1]; to[2]   = next_yaw;

    if(singul == -1){
      from[2] = cur_state->yaw + M_PI;
      if(from[2] > M_PI)
        from[2] -= 2 * M_PI;

      to[2] = next_yaw + M_PI;
      if(to[2] > M_PI)
        to[2] -= 2 * M_PI;
    }

      //  dubins_curve_->interpolate(from(), to(), max_vel_ * step_time / len, s());
      // auto reals = s.reals();
    Eigen::Vector3d xt;
    std::vector<double> reals;
    double dis = dubins_curve_->distance(from(), to());
    int check_num_car = ceil(dis / 0.01);
    
    
    Eigen::VectorXd delta_theta = next_state.tail(manipulator_dof_) - cur_state->state.tail(manipulator_dof_);
    double max_delta_theta = delta_theta.lpNorm<Eigen::Infinity>();
    int check_num_theta = ceil(max_delta_theta / 0.01);
    int piece_num_temp = std::max(check_num_car, check_num_theta);
    piece_num_temp = std::max(piece_num_temp, 10);
    for(int i = 0; i < piece_num_temp; ++i){
      double temp_i = (double)i / (double)piece_num_temp;
      dubins_curve_->interpolate(from(), to(), temp_i, s());// 获得对应长度的中间点
      reals = s.reals();
      if(singul == -1){
        reals[2] = reals[2] + M_PI;
        if(reals[2] > M_PI)
          reals[2] -= 2 * M_PI;
      }
      xt = Eigen::Vector3d(reals[0], reals[1], reals[2]);
      if(mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_) + delta_theta * temp_i, false)){
        return true;
      }
    }
    // if(check_num < 10)
    //   check_num = 10;
    // for (int i = 0; i <= check_num; ++i){
    //   dubins_curve_->interpolate(from(), to(), (double)i / (double)check_num, s());
    //   reals = s.reals();
    //   xt[0] = reals[0]; xt[1] = reals[1]; xt[2] = reals[2]; 
    //   if(singul == -1){
    //     xt[2] = reals[2] + M_PI;
    //     if(xt[2] > M_PI)
    //       xt[2] -= 2 * M_PI;
    //   }
      
    //   if(mm_config_->checkcollision(xt, (cur_state->state.tail(manipulator_dof_) + (next_state.tail(manipulator_dof_) - cur_state->state.tail(manipulator_dof_)) * double(i) / double(check_num)), false)){
    //     return true;
    //   }
    // }

    return false;
  }

  bool RrtPlanning::checkcollision(PathNodeRRTPtr& cur_state, PathNodeRRTPtr& next_state){
    if(cur_state->node_state == PathNodeRRT::NODE_STATE::COLLISION || next_state->node_state == PathNodeRRT::NODE_STATE::COLLISION){
      return true;
    }
    // ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    // from[0] = cur_state->state[0]; from[1] = cur_state->state[1]; from[2] = cur_state->yaw;
    // to[0]   = next_state->state[0]; to[1]  = next_state->state[1]; to[2] = next_state->yaw;

    //   //  dubins_curve_->interpolate(from(), to(), max_vel_ * step_time / len, s());
    //   // auto reals = s.reals();
    // Eigen::Vector3d xt;
    // std::vector<double> reals;
    // double dis = dubins_curve_->distance(from(), to());
    // int check_num = ceil(dis / 0.05);
    //     if(check_num < check_num_)
    //   check_num = check_num_;
    // for (int i = 0; i <= check_num; ++i){
    //   dubins_curve_->interpolate(from(), to(), (double)i / (double)check_num, s());
    //   reals = s.reals();
    //   xt[0] = reals[0]; xt[1] = reals[1]; xt[2] = reals[2]; 
      
    //   if(mm_config_->checkManicollision(xt, (cur_state->state.tail(manipulator_dof_) + (next_state->state.tail(manipulator_dof_) - cur_state->state.tail(manipulator_dof_)) * double(i) / double(check_num)), false)){
    //     return true;
    //   }
    // }
    // return false;
    return checkcollision(cur_state, next_state->state, next_state->yaw);
  
  }

  void RrtPlanning::smoothTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> &yaw_list, std::vector<double> &t_list){
    PathNodeRRTPtr temp_end = end_node_;
    PathNodeRRTPtr last = nullptr, temp;
    int traj_num = traj.size();
    for(int i = 0; i < traj_num; ++i){
      temp = initNode(traj[i], yaw_list[i]);
      temp->parent = last;
      last = temp;
    }
    end_node_ = last;
    have_path_ = true;
    getTraj(traj, yaw_list, t_list);
    end_node_ = temp_end;
    return;
  }

  bool RrtPlanning::smooth(PathNodeRRTPtr smooth_node){
    if(!have_path_)
    {
      return false;
    }
    PathNodeRRTPtr node = smooth_node;
    PathNodeRRTPtr temp, new_node;
    while(node != nullptr){
      temp = node->parent;
      while(temp != nullptr){
        if(!checkcollision(temp, node)){
          linkNode(temp, node);
        }
        temp = temp->parent;
      }
      node = node->parent;
    }
    // double des_time = 0.4;

    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
    double t;
    int rel;
    Eigen::VectorXd temp_state(traj_dim_);
    double temp_yaw;
    node = smooth_node;
    while(node != nullptr){
      temp = node->parent;
      if(temp != nullptr){
        t = getTime(temp, node);

        rel = floor(t / time_resolution_);
        from[0] = node->parent->state[0]; from[1] = node->parent->state[1]; from[2] = node->parent->yaw;
        to[0] = node->state[0]; to[1] = node->state[1]; to[2] = node->yaw;

        for(int i = 1; i < rel; ++i){
          dubins_curve_->interpolate(from(), to(), (double)i / rel, s());
          temp_state.tail(manipulator_dof_) = temp->state.tail(manipulator_dof_) + (node->state.tail(manipulator_dof_) - temp->state.tail(manipulator_dof_)) * (double)i / (double)rel;
          temp_state[0] = s[0]; temp_state[1] = s[1];
          if(node->singul == 1)
            temp_yaw = s[2];
          else{
            temp_yaw = s[2] + M_PI;
            if(temp_yaw > M_PI)
              temp_yaw -= 2 * M_PI;
          }
          new_node = initNode(temp_state, temp_yaw);
          linkNode(node->parent, new_node);
          linkNode(new_node, node);
        }
      }
      node = temp;
    }
    return true;
  }


  bool RrtPlanning::getTraj(std::vector<Eigen::VectorXd> &traj, std::vector<double> &yaw_list, std::vector<double> &t_list){
    traj.clear();
    yaw_list.clear();
    t_list.clear();
    if(!have_path_) return false;
    PathNodeRRTPtr node = end_node_;
    node = end_node_;
    while(node != nullptr){
      smooth(node);
      node = node->parent;
    }

    node = end_node_;
    Eigen::VectorXd temp_state(traj_dim_);
    while(node != nullptr){
      traj.push_back(node->state);
      yaw_list.push_back(node->yaw);
      if(node->parent != nullptr)
        t_list.push_back(getTime(node->parent, node));
      node = node->parent;
    }

    reverse(traj.begin(), traj.end());
    reverse(yaw_list.begin(), yaw_list.end());
    reverse(t_list.begin(), t_list.end());
    return true;
  }

  double RrtPlanning::getTime(PathNodeRRTPtr &pre_node, PathNodeRRTPtr &cur_node){
    double t = -1.0;
    Eigen::VectorXd pre_state = pre_node->state;
    Eigen::VectorXd cur_state = cur_node->state;

    ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_);
    from[0] = pre_node->state[0]; from[1] = pre_node->state[1]; from[2] = pre_node->yaw;
    to[0]   = cur_node->state[0]; to[1]   = cur_node->state[1]; to[2]   = cur_node->yaw;
    if(cur_node->singul == -1){
      from[2] = pre_node->yaw + M_PI;
      if(from[2] > M_PI) from[2] -= 2 * M_PI;
      
      to[2] = cur_node->yaw + M_PI;
      if(to[2] > M_PI) to[2] -= 2 * M_PI;
    }

    double len = dubins_curve_->distance(from(), to());

    t = max(len / max_vel_, t);

    for(int i = 0; i < manipulator_dof_; ++i){
      t = max(calAngleErr(pre_state(mobile_base_dof_ + i) , cur_state(mobile_base_dof_ + i)) / max_joint_vel_, t);
    }
    return t;
  }

  double RrtPlanning::getCost(){
    if(!have_path_) return -1.0;
    return end_node_->g_score;
  }

  bool RrtPlanning::getLayer(int& start_layer, int& end_layer){
    if(!have_path_) return false;
    end_layer = end_node_->layer;
    PathNodeRRTPtr node = end_node_;
    while(node != nullptr){
      if(node->parent == nullptr){
        start_layer = node->layer;
        break;
      }
      node = node->parent;
    }
    return true;
  }

  int RrtPlanning::getPathLen(){
    if(!have_path_) return -1;
    PathNodeRRTPtr temp = end_node_;
    int len = 0;
    while(temp != nullptr){
      ++len;
      temp = temp->parent;
    }
    return len;
  }

  void RrtPlanning::reset(){
    this->max_index_ = 0;
    this->have_path_ = false;
    this->tree_count_ = 0;
    this->anti_tree_count_ = 0;
    this->end_node_ = nullptr;

    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      delete it->second;
    }
    node_pool_.clear();
  }

  void RrtPlanning::setParam(ros::NodeHandle& nh, const std::shared_ptr<MMConfig> &mm_config){
    mm_config_ = mm_config;
    manipulator_link_pts_ = mm_config_->getLinkPoint();
    max_vel_ = mm_config_->getBaseMaxVel();
    T_q_0_ = mm_config_->getTq0();

    nh.param("search/check_num", check_num_, -1);
    nh.param("search/goal_rate", goal_rate_, 0.0);
    nh.param("search/max_sample_time", max_sample_time_, 0.0);
    nh.param("search/max_loop_num", max_loop_num_, 100);
    nh.param("search/time_resolution", time_resolution_, 1.0);
    nh.param("optimization/self_safe_margin", self_safe_margin_, 0.1);
    nh.param("optimization/safe_margin_mani", safe_margin_mani_, 0.1);

    nh.param("mm/mobile_base_dof", mobile_base_dof_, -1);

    double dist_resolution;
    nh.param("search/dist_resolution", dist_resolution, 1.0);
    time_resolution_ = dist_resolution / max_vel_;

    std::vector<double> joint_pos_limit;
    nh.getParam("mm/manipulator_min_pos", joint_pos_limit);
    min_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        min_joint_pos_(i) = joint_pos_limit[i];
    }

    joint_pos_limit.clear();
    nh.getParam("mm/manipulator_max_pos", joint_pos_limit);
    max_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        max_joint_pos_(i) = joint_pos_limit[i];
    }
    nh.param("mm/manipulator_max_vel", max_joint_vel_, -1.0);
    nh.param("mm/manipulator_max_acc", max_joint_acc_, -1.0);
    nh.param("mm/manipulator_dof", manipulator_dof_, -1);
    nh.param("mm/manipulator_thickness", mani_thickness_, 0.1);

    dubins_curve_ = std::make_shared<ompl::base::DubinsStateSpace>(0.1);
    traj_dim_ = mobile_base_dof_ + manipulator_dof_;

    for(int i = 0; i < mobile_base_dof_; ++i){
      this->max_size_.push_back(0.0);
      this->min_size_.push_back(0.0);
    }
    for(int i = 0 ; i < manipulator_dof_; ++i){
      this->max_size_.push_back(max_joint_pos_[i]);
      this->min_size_.push_back(min_joint_pos_[i]);
    }
    // yaw
    this->max_size_.push_back(M_PI);
    this->min_size_.push_back(-M_PI);
  }
}  //namespace remani_planner