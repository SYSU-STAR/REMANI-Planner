#include "path_searching/sample_mani_RRT.h"

namespace mani_sample{
  bool SampleMani::sampleManiSearch(const bool astar_succ, const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state,
                    const std::vector<Eigen::Vector3d> &car_state_list, const std::vector<Eigen::Vector3d> &car_state_list_check, 
                    const std::vector<double> &t_list, const std::vector<int> &singul_container, const int start_singul,// size = t_list.size()
                    // output
                    std::vector<std::vector<Eigen::VectorXd>> &simple_path_container, std::vector<int> &singul_container_new,
                    std::vector<std::vector<double>> &yaw_list_container, std::vector<Eigen::VectorXd> &t_list_container){
    simple_path_container.clear();
    singul_container_new.clear();
    yaw_list_container.clear();
    t_list_container.clear();
    
    std::vector<Eigen::VectorXd> simple_path; // x, y, theta
    std::vector<double> yaw_list, t_list_temp;
    Eigen::VectorXd t_vector;
    std::vector<Eigen::VectorXd> mani_path;
    Eigen::VectorXd state_full(traj_dim_);
    init(car_state_list, car_state_list_check, t_list);
    bool mani_status = search(start_state, end_state);

    // ROS_ERROR("=====================1");
    if(mani_status && astar_succ){
      getTraj(mani_path);
      // ROS_ERROR("=====================1");
      int singul_now = singul_container[0];
      state_full.head(mobile_base_dof_) = car_state_list[0].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = mani_path[0];
      simple_path.push_back(state_full);
      yaw_list.push_back(car_state_list[0](2));
      // ROS_ERROR("=====================2");
      // for(int i = 0; i < singul_container.size(); ++i){
      //   std::cout << "singul" << i << ": " << singul_container[i] << std::endl;
      // }
      for(int i = 1; i < max_index_; ++i){
        if(singul_now != singul_container[i - 1]){
          singul_container_new.push_back(singul_now);
          singul_now = singul_container[i - 1];
          // ROS_ERROR("=====================3");
          // rrt_plan_->smoothTraj(simple_path, yaw_list, t_list_temp);
          simple_path_container.push_back(simple_path);
          yaw_list_container.push_back(yaw_list);
          t_vector.resize(t_list_temp.size());
          for(unsigned int j = 0; j < t_list_temp.size(); ++j){
            t_vector[j] = t_list_temp[j];
          }
          t_list_container.push_back(t_vector);
          simple_path.clear();
          yaw_list.clear();
          t_list_temp.clear();

          simple_path.push_back(state_full);
          yaw_list.push_back(car_state_list[i - 1](2));
        }
        // ROS_ERROR("=====================4");
        state_full.head(mobile_base_dof_) = car_state_list[i].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = mani_path[i];
        simple_path.push_back(state_full);
        yaw_list.push_back(car_state_list[i](2));
        t_list_temp.push_back(t_list[i - 1]);
      }

      // rrt_plan_->smoothTraj(simple_path, yaw_list, t_list_temp);
      singul_container_new.push_back(singul_now);
      simple_path_container.push_back(simple_path);
      yaw_list_container.push_back(yaw_list);
      t_vector.resize(t_list_temp.size());
      for(unsigned int j = 0; j < t_list_temp.size(); ++j){
        t_vector[j] = t_list_temp[j];
      }
      t_list_container.push_back(t_vector);

      // std::cout << "path: " << std::endl;
      // for(int j = 0; j < path.size(); ++j){
      //   std::cout << path[j].transpose() << std::endl;
      // }
      // std::cout << "t_list: " << std::endl;
      // for(int j = 0; j < t_list_new.size(); ++j){
      //   std::cout << t_list_new[j] << " ";
      // }
      // std::cout << std::endl;
      
      return true;
    }
    // tree_max_index_ = min(tree_max_index_, 3);
    // tree_min_index_ = max(tree_min_index_, min(max_index_ - 1, 6));
    
    if(!astar_succ){
      if(end_node_ != nullptr)
        end_node_->parent = nullptr;
    }

    // std::cout << "all: " << std::endl;
    // for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
    //   std::cout << it->second->state.transpose() << std::endl;
    // }
    // ROS_ERROR("[Sample Mani]:fail !!! Try to fix.");
    // std::cout << "test 1" << std::endl; 
    // 出意外了，需要补上中间的
    std::vector<Eigen::VectorXd> start_list, end_list;
    std::vector<double> start_yaw_list, end_yaw_list, start_g_score_list, end_g_score_list;
    std::vector<int> start_layer_list, end_layer_list, start_singul_list, end_singul_list;

    // 寻找那些需要放进去的层数，扔到rrt的起点和终点里
    int index_front, index_last;
    // std::cout << "test 2" << std::endl; 
    // std::cout << "tree_max_index: " << tree_max_index_ << std::endl;
    // std::cout << "tree_min_index: " << tree_min_index_ << std::endl;
    if(tree_max_index_ <= tree_min_index_){
      index_front = max(tree_max_index_ - 1, 0);
      index_last = tree_max_index_;
    }
    else{
      index_front = max(tree_min_index_ - 1, 0);
      index_last = tree_max_index_;
    }
    // std::cout << "test 2.1" << std::endl;
    // std::cout << "index_front: " << index_front << std::endl;
    // std::cout << "index_last: " << index_last << std::endl;
    // index_front = index_last = 0;

    auto it = node_pool_.lower_bound(string(1, index_front));
    for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
      if(it->second->node_state == ManiPathNode::NODE_STATE::IN_TREE){
        state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = it->second->state;
        start_list.push_back(state_full);
        start_yaw_list.push_back(car_state_list[it->second->index](2));
        start_g_score_list.push_back(it->second->g_score);
        start_layer_list.push_back(it->second->index);
        if(it->second->index == 0)
          start_singul_list.push_back(start_singul);
        else
          start_singul_list.push_back(singul_container[it->second->index - 1]);
      }
    }

    // if(index_front > 0){
    //   index_front = index_last = 0;
    //   auto it = node_pool_.lower_bound(string(1, index_front));
    //   for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
    //     if(it->second->node_state == ManiPathNode::NODE_STATE::IN_TREE){
    //       state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
    //       state_full.tail(manipulator_dof_) = it->second->state;
    //       start_list.push_back(state_full);
    //       start_yaw_list.push_back(car_state_list[it->second->index](2));
    //       start_g_score_list.push_back(it->second->g_score);
    //       start_layer_list.push_back(it->second->index);
    //       if(it->second->index == 0)
    //         start_singul_list.push_back(start_singul);
    //       else
    //         start_singul_list.push_back(singul_container[it->second->index - 1]);
    //     }
    //   }
    // }
    // std::cout << "start_size: " << start_list.size() << std::endl;

    if(tree_max_index_ <= tree_min_index_){
      index_front = tree_min_index_;
      index_last = min(tree_min_index_ + 1, max_index_ - 1);
    }
    else{
      index_front = tree_min_index_;
      index_last = min(tree_max_index_ + 1, max_index_ - 1);
    }
    // std::cout << "test 2.2" << std::endl;
    // std::cout << "index_front: " << index_front << std::endl;
    // std::cout << "index_last: " << index_last << std::endl;
    // index_front = index_last = max_index_ - 1;
    it = node_pool_.lower_bound(string(1, index_front));
    for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
      if(it->second->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE){
        state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
        state_full.tail(manipulator_dof_) = it->second->state;
        end_list.push_back(state_full);
        end_yaw_list.push_back(car_state_list[it->second->index](2));
        end_g_score_list.push_back(it->second->g_score);
        end_layer_list.push_back(it->second->index);
        if(it->second->index == max_index_ - 1)
          end_singul_list.push_back(0);
        else
          end_singul_list.push_back(-singul_container[it->second->index]);
      }
    }

    // if(index_last < max_index_ - 1){
    //   index_front = index_last = max_index_ - 1;
    //   it = node_pool_.lower_bound(string(1, index_front));
    //   for(; it != node_pool_.end() && it->second->index <= index_last; ++it){
    //     if(it->second->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE){
    //       state_full.head(mobile_base_dof_) = car_state_list[it->second->index].head(mobile_base_dof_);
    //       state_full.tail(manipulator_dof_) = it->second->state;
    //       end_list.push_back(state_full);
    //       end_yaw_list.push_back(car_state_list[it->second->index](2));
    //       end_g_score_list.push_back(it->second->g_score);
    //       end_layer_list.push_back(it->second->index);
    //       if(it->second->index == max_index_ - 1)
    //         end_singul_list.push_back(0);
    //       else
    //         end_singul_list.push_back(-singul_container[it->second->index]);
    //     }
    //   }
    // }
    // std::cout << "end_size: " << end_list.size() << std::endl;

    // std::cout << "test 3" << std::endl; 
    // rrt搜
    std::vector<Eigen::VectorXd> path_fill;
    std::vector<double> yaw_list_fill, t_list_fill;
    bool status = rrt_plan_->RRTSearchAndGetSimplePath(start_list, start_yaw_list, end_list, end_yaw_list, 
                                                      start_g_score_list, start_layer_list, end_g_score_list, end_layer_list,
                                                      start_singul_list, end_singul_list,
                                                      path_fill, yaw_list_fill, t_list_fill);
    // std::cout << "test 4" << std::endl; 
    if(!status){
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }
      
    // std::cout << "test 5" << std::endl; 

    // 找到中间这段轨迹的起点终点在分层规划中的对应节点
    Eigen::Vector3d short_path_start, short_path_end;
    Eigen::VectorXd short_mani_start(manipulator_dof_), short_mani_end(manipulator_dof_);
    int path_fill_num = path_fill.size();

    short_path_start.head(2) = path_fill[0].head(2);
    short_path_start(2) = yaw_list_fill[0];
    short_mani_start = path_fill[0].tail(manipulator_dof_);

    short_path_end.head(2) = path_fill[path_fill_num-1].head(2);
    short_path_end(2) = yaw_list_fill[path_fill_num-1];
    short_mani_end = path_fill[path_fill_num-1].tail(manipulator_dof_);

    int part_idx_begin = -1, part_idx_end = -1;
    rrt_plan_->getLayer(part_idx_begin, part_idx_end);
    // std::cout << "start layer: " << part_idx_begin << std::endl;
    // std::cout << "end layer: " << part_idx_end << std::endl;
    // for(int i = 0; i < max_index_; ++i){
    //   if((car_state_list[i] - short_path_start).norm() < 1.0e-3 && node_pool_.find(calculateValue(i, coord2idx(short_mani_start))) != node_pool_.end()){
    //     part_idx_begin = i;
    //   }
    //   if((car_state_list[i] - short_path_end).norm() < 1.0e-3 && node_pool_.find(calculateValue(i, coord2idx(short_mani_end))) != node_pool_.end()){
    //     part_idx_end = i;
    //   }
    // }
    // std::cout << "test 6" << std::endl; 

    ManiPathNodePtr q_t = nullptr, q_an_t = nullptr, q_temp;
    // std::cout << "part_idx_begin: " << part_idx_begin << std::endl;
    // std::cout << "short_mani_start: " << short_mani_start.transpose() << std::endl;
    it = node_pool_.find(calculateValue(part_idx_begin, short_mani_start));
    if(it != node_pool_.end()){
      q_t = it->second;
    }else{
      ROS_ERROR("find part_idx_begin, short_mani_start fail!");
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }
    it = node_pool_.find(calculateValue(part_idx_end, short_mani_end));
    if(it != node_pool_.end()){
      q_an_t = it->second;
    }else{
      ROS_ERROR("find part_idx_end, short_mani_end fail!");
      goal_gen_.seed(std::random_device{}());
      state_gen_.seed(std::random_device{}());
      node_gen_.seed(std::random_device{}());
      return false;
    }
    // std::cout << "test 7" << std::endl; 
    // 把节点都串起来
    std::vector<Eigen::VectorXd> state_vector_temp;
    std::vector<double> t_vector_temp, yaw_vector_temp;
    std::vector<int> singul_vector_temp;
    q_temp = q_t;
    while(q_temp != nullptr){
      state_full.head(mobile_base_dof_) = car_state_list[q_temp->index].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = q_temp->state;
      state_vector_temp.push_back(state_full);
      yaw_vector_temp.push_back(car_state_list[q_temp->index](2));
      if(q_temp->index > 0){
        t_vector_temp.push_back(t_list[q_temp->index - 1]);
        // singul_vector_temp.push_back(singul_container[q_temp->index - 1]);
        singul_vector_temp.push_back(1);
      }
      q_temp = q_temp->parent;
    }
    // std::cout << "test 8" << std::endl; 
    std::reverse(state_vector_temp.begin(), state_vector_temp.end());
    std::reverse(t_vector_temp.begin(), t_vector_temp.end());
    std::reverse(singul_vector_temp.begin(), singul_vector_temp.end());
    std::reverse(yaw_vector_temp.begin(), yaw_vector_temp.end());
    // std::cout << "test 9" << std::endl; 
    t_vector_temp.push_back(t_list_fill[0]);
    int singul;
    if(singul_vector_temp.size() == 0)
      singul = (path_fill[1] - path_fill[0]).head(2).dot(Eigen::Vector2d(cos(yaw_list_fill[0]), sin(yaw_list_fill[0]))) >= 0 ? 1 : -1;
    else
      singul = singul_vector_temp.back();
    // singul_vector_temp.push_back(singul);
    singul_vector_temp.push_back(1);
    for(int i = 1; i < path_fill_num - 1; ++i){
      // singul = (path_fill[i + 1] - path_fill[i]).head(2).dot(Eigen::Vector2d(cos(yaw_list_fill[i]), sin(yaw_list_fill[i]))) >= 0 ? 1 : -1;
      singul = (path_fill[i + 1] - path_fill[i]).head(2).dot((path_fill[i] - path_fill[i - 1]).head(2)) >= 0 ? singul : -singul;
      // singul_vector_temp.push_back(singul);
      singul_vector_temp.push_back(1);
      state_vector_temp.push_back(path_fill[i]);
      yaw_vector_temp.push_back(yaw_list_fill[i]);
      t_vector_temp.push_back(t_list_fill[i]);
    }
    q_temp = q_an_t;
    while(q_temp != nullptr){
      state_full.head(mobile_base_dof_) = car_state_list[q_temp->index].head(mobile_base_dof_);
      state_full.tail(manipulator_dof_) = q_temp->state;
      state_vector_temp.push_back(state_full);
      yaw_vector_temp.push_back(car_state_list[q_temp->index](2));
      if(q_temp->index < max_index_ - 1){
        t_vector_temp.push_back(t_list[q_temp->index]);
        singul_vector_temp.push_back(1);
      }
      q_temp = q_temp->parent;
    }

    simple_path.clear();
    yaw_list.clear();
    t_list_temp.clear();
    int singul_now = singul_vector_temp[0];
    simple_path.push_back(state_vector_temp[0]);
    yaw_list.push_back(yaw_vector_temp[0]);

    for(unsigned int i = 1; i < state_vector_temp.size(); ++i){
      // std::cout << "state: " << state_vector_temp[i].transpose() << "\n";
      // std::cout << "singul: " << singul_vector_temp[i - 1] << "\n";
      if(singul_now != singul_vector_temp[i - 1]){
        singul_container_new.push_back(singul_now);
        singul_now = singul_vector_temp[i - 1];

        simple_path_container.push_back(simple_path);
        yaw_list_container.push_back(yaw_list);
        t_vector.resize(t_list_temp.size());
        for(unsigned int j = 0; j < t_list_temp.size(); ++j){
          t_vector[j] = t_list_temp[j];
        }
        t_list_container.push_back(t_vector);
        simple_path.clear();
        yaw_list.clear();
        t_list_temp.clear();

        simple_path.push_back(state_vector_temp[i - 1]);
        yaw_list.push_back(yaw_vector_temp[i - 1]);
      }

      simple_path.push_back(state_vector_temp[i]);
      yaw_list.push_back(yaw_vector_temp[i]);
      t_list_temp.push_back(t_vector_temp[i - 1]);
    }

    // rrt_plan_->smoothTraj(simple_path, yaw_list, t_list_temp);
    singul_container_new.push_back(singul_now);
    simple_path_container.push_back(simple_path);
    yaw_list_container.push_back(yaw_list);
    t_vector.resize(t_list_temp.size());
    for(unsigned int j = 0; j < t_list_temp.size(); ++j){
      t_vector[j] = t_list_temp[j];
    }
    t_list_container.push_back(t_vector);

    //TODO singul
    // singul_container.clear();
    // int singul = 1;
    // Eigen::Vector2d cur_dir(cos(yaw_list[0]), sin(yaw_list[0]));
    // Eigen::Vector2d next_dir;
    // int path_num = path.size();
    // for(int i = 0; i < path_num - 1; ++i){
    //   next_dir = ((path[i+1] - path[i]).head(2)).normalized();
    //   if(cur_dir.dot(next_dir) < 0){
    //     singul = -singul;
    //   }
    //   singul_container.push_back(singul);
    //   cur_dir = next_dir;
    // }

    return true;
  }

  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){
    // std::cout << "[sample mani]: Search begin. start: " << start_state.transpose() << " end: " << end_state.transpose() << std::endl;
    ros::Time time_1 = ros::Time::now();
    std::uniform_real_distribution<double> goal_dis(0.0, 1.0);
    // std::uniform_int_distribution<int> node_dis(1, max_index_-1);
    ManiPathNodePtr start_node, end_node;
    // std::cout << "++++++++++++++++++++++++1" << std::endl;
    if(!node_pool_.empty()){
      delete node_pool_.begin()->second;
      node_pool_.erase(node_pool_.begin());
    }
    start_node = new ManiPathNode(manipulator_dof_);
    start_node->index = 0;
    start_node->state = start_state;
    start_node->node_state = ManiPathNode::NODE_STATE::IN_TREE;
    start_node->g_score = 0.0;
    // std::cout << "test: " << node_pool_.begin()->second->index << std::endl;
    // std::cout << "++++++++++++++++++++++++2" << std::endl;
    for(auto it = node_pool_.begin(); it != node_pool_.end() && it->second->index == 1; ++it)
    {
      it->second->parent = start_node;
      start_node->children.insert(std::make_pair(calculateValue(it->second), it->second));
    }
    node_pool_.insert(std::make_pair(calculateValue(start_node), start_node));
    // std::cout << "++++++++++++++++++++++++2.1" << std::endl;
    
    organizeTree(start_node);
    // ROS_INFO("[Sample Mani]: Init1 organ. num_in_tree: %d, node expend: %d, time: %lf.", tree_count_, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
    time_1 = ros::Time::now();

    tree_min_index_ = max_index_ - 1;
    end_node = initNode(max_index_ - 1, end_state);
    if(end_node->node_state == ManiPathNode::NODE_STATE::IN_TREE){
      this->end_node_ = end_node;
      have_path_ = true;
      // ROS_WARN("[Sample Mani]: Success. num_in_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, 0, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
      return true;
    }
    end_node->node_state = ManiPathNode::NODE_STATE::IN_ANTI_TREE;
    end_node->g_score = 0.0;
    ++anti_tree_count_;

    // std::cout << "setup: " << std::endl;
    // auto it_1111 = node_pool_.begin();
    // for(; it_1111 != node_pool_.end(); ++it_1111){
    //   std::cout << it_1111->second->index << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "shu: " << std::endl;
    // it_1111 = node_pool_.begin();
    // for(; it_1111 != node_pool_.end(); ++it_1111){
    //   std::cout << it_1111->second->node_state << " ";
    // }
    // std::cout << std::endl;

    if(max_index_ == 2){
      // end_node->parent = start_node;
      // this->have_path_ = true;
      // this->end_node_ = end_node;
      // return true;
      if(!checkcollision(start_node, end_node)){
        have_path_ = true;
        linkNode(start_node, end_node);
        // end_node->node_state = ManiPathNode::NODE_STATE::IN_TREE;
        this->end_node_ = end_node;
        return true;
      }
      return false;

    }
    // std::cout << "++++++++++++++++++++++++4" << std::endl;
    int loop = 0;
    ManiPathNodePtr q_rand, q_new, q_near;
    ManiPathNodePtr q_new_1, q_near_1;
    ManiPathNodePtr q_new_2;
    ManiPathNodePtr path_node_1 = nullptr, path_node_2 = nullptr;
    double c_min = 1.0e6;

    bool dir = false;
    auto t_start = ros::Time::now();
    // while(loop < max_loop_num_){
    while(true){
      if((ros::Time::now() - t_start).toSec() > max_mani_search_time_){
        break;
      }


      if(tree_count_ > anti_tree_count_){
        dir = false;
      }
      else{
        dir = true;
      }
      ++loop;
      if(goal_dis(goal_gen_) < goal_rate_)
      {
        if(dir)
          q_rand = end_node;
        else
          q_rand = start_node;
      }
      else
      {
        q_rand = getSampleNode();
      }
      if(q_rand == nullptr){
        continue;
      }

      q_near = getNearestNode(q_rand, dir);
      if(q_near == nullptr){
        continue;
      }

      q_new = extendNode(q_near, q_rand, dir);
      if(q_new == nullptr){
        continue;
      }
      
      if((dir && q_new->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE) || (!dir && q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
        if(q_new->g_score + q_near->g_score + estimateHeuristic(q_new, q_near) < c_min){
          have_path_ = true;
          c_min = q_new->g_score + q_near->g_score + estimateHeuristic(q_new, q_near);
          path_node_1 = q_new;
          path_node_2 = q_near;
          // std::cout << "min: " << c_min << std::endl;
        } 
        // mergeTrees(q_new, q_near);
        // ROS_WARN("[Sample Mani]: Success. num_in_tree: %d, node expend: %d, time: %lf.", tree_count_, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
        // return true;
        continue;
      }

      linkNode(q_near, q_new);
      q_new->node_state = q_near->node_state;
      adjustTree(q_new, dir);
      if(q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE)
        ++tree_count_;
      else
        ++anti_tree_count_;

      if(q_new->node_state == ManiPathNode::NODE_STATE::IN_TREE && q_new->index > tree_max_index_){
        tree_max_index_ = q_new->index;
      }
      else if(q_new->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE && q_new->index < tree_min_index_){
        tree_min_index_ = q_new->index;
      }

      // if(q_new->index == max_index_ - 1)
      // {
      //   this->end_node_ = q_new;
      //   have_path_ = true;
      //   ROS_INFO("[Sample Mani]: Success. num_in_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, loop, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
      //   return true;
      // }

      q_near_1 = getNearestNode(q_new, !dir);
      if(q_near_1 == nullptr)
        continue;
      q_new_1 = extendNode(q_near_1, q_new, !dir);
      if(q_new_1 == nullptr)
        continue;
      
      if((!dir && q_new_1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE) || (dir && q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
        if(q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1) < c_min){
          have_path_ = true;
          c_min = q_new_1->g_score + q_near_1->g_score + estimateHeuristic(q_new_1, q_near_1);
          path_node_1 = q_new_1;
          path_node_2 = q_near_1;
          // std::cout << "min: " << c_min << std::endl;
        } 
        // mergeTrees(q_new_1, q_near_1);
        // ROS_WARN("[Sample Mani]: Success. num_in_tree: %d, node expend: %d, time: %lf.", tree_count_, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
        // return true;
        continue;
      }

      linkNode(q_near_1, q_new_1);
      q_new_1->node_state = q_near_1->node_state;

      if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE && q_new_1->index > tree_max_index_)
      {
        tree_max_index_ = q_new_1->index;
      }
      else if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE && q_new_1->index < tree_min_index_){
        tree_min_index_ = q_new_1->index;
      }
      if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)
        ++tree_count_;
      else
        ++anti_tree_count_;
      adjustTree(q_new_1, !dir);

      while(q_new_1->index != q_new->index){
        q_new_2 = extendNode(q_new_1, q_new, !dir);
        if(q_new_2 != nullptr){

          if((!dir && q_new_2->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE) || (dir && q_new_2->node_state == ManiPathNode::NODE_STATE::IN_TREE)){
            if(q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1) < c_min){
              have_path_ = true;
              c_min = q_new_2->g_score + q_new_1->g_score + estimateHeuristic(q_new_2, q_new_1);
              path_node_1 = q_new_2;
              path_node_2 = q_new_1;
              // std::cout << "min: " << c_min << std::endl;
            } 
            // mergeTrees(q_new_2, q_new_1);
            // ROS_WARN("[Sample Mani]: Success. num_in_tree: %d, node expend: %d, time: %lf.", tree_count_, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
            // return true;
            break;
          }

          linkNode(q_new_1, q_new_2);
          q_new_2->node_state = q_new_1->node_state;
          if(q_new_2->node_state == ManiPathNode::NODE_STATE::IN_TREE && q_new_2->index > tree_max_index_)
          {
            tree_max_index_ = q_new_2->index;
          }
          else if(q_new_2->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE && q_new_2->index < tree_min_index_){
            tree_min_index_ = q_new_2->index;
          }

          if(q_new_1->node_state == ManiPathNode::NODE_STATE::IN_TREE)
            ++tree_count_;
          else
            ++anti_tree_count_;

          q_new_2->node_state = q_new_1->node_state;
          q_new_1 = q_new_2;
          // std::cout << "q_new_1: " << q_new_1->index << std::endl;
        }
        else{
          break;
        }
      }
    }
    // ROS_ERROR("[Sample Mani]: Fail. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
    // std::cout << "tree: " << tree_count_ << " max: " << tree_max_index_ <<std::endl;
    // for(auto &node : node_pool_)
    //   if(node.second->node_state == ManiPathNode::NODE_STATE::IN_TREE)
    //         std::cout << node.second->index << " ";
    // std::cout << std::endl << std::endl;

    // std::cout << "anti tree: " << anti_tree_count_ << " min: " << tree_min_index_ << std::endl;
    // for(auto &node : node_pool_)
    //   if(node.second->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE)
    //         std::cout << node.second->index << " ";
    // std::cout << std::endl << std::endl;
    
    if(have_path_){
      mergeTrees(path_node_1, path_node_2);
      // ROS_INFO("[Sample Mani]: Success. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, (int)node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
    }else{
      // ROS_ERROR("[Sample Mani]: Fail. num_in_tree: %d, num_in_anti_tree: %d, loop num: %d, node expend: %d, time: %lf.", tree_count_, anti_tree_count_, loop, (int)node_pool_.size(), (ros::Time::now() - time_1).toSec() * 1000.0);
    }
      
    return have_path_;
  }

  ManiPathNodePtr SampleMani::initNode(int idx, const Eigen::VectorXd &s){
    string key = calculateValue(idx, s);
    auto it = node_pool_.find(key);
    if(it != node_pool_.end()){
      return it->second;
    }
    ManiPathNodePtr node = new ManiPathNode;
    node->index = idx;
    node->node_state = ManiPathNode::NODE_STATE::EXPAND;
    node->state = s;
    
    if(mm_config_->checkManicollision(car_state_list_[node->index], node->state, false))
    {
      node->node_state = ManiPathNode::NODE_STATE::COLLISION;
    }
    node_pool_.insert(std::make_pair(key, node));
    return node;
  }

  ManiPathNodePtr SampleMani::getSampleNode(){
    std::uniform_int_distribution<int> node_dis(1, max_index_-2);
    std::vector<std::uniform_real_distribution<double>> state_dis_vec;
    state_dis_vec.reserve(manipulator_dof_);
    for(int i = 0; i < manipulator_dof_; ++i){
      std::uniform_real_distribution<double> state_dis(min_joint_pos_[i], max_joint_pos_[i]);
      state_dis_vec.push_back(state_dis);
    }
    Eigen::VectorXd sample_state(manipulator_dof_);
    ManiPathNodePtr sample_node = nullptr;
    int sample_idx;
    int count = 0;
    while(true)
    {
      if(count > 20)
      {
        return nullptr;
      }
      ++count;
      
      sample_idx = node_dis(node_gen_);
      for(int i = 0; i < manipulator_dof_; ++i)
      {
        sample_state(i) = state_dis_vec[i](state_gen_);
      }
      sample_node = initNode(sample_idx, sample_state);
      if(sample_node->node_state == ManiPathNode::NODE_STATE::COLLISION)
      {
        continue;
      }
      break;
    }
    return sample_node;
  }

  ManiPathNodePtr SampleMani::getNearestNode(ManiPathNodePtr &x, bool dir){
    int pre_idx;
    if(dir){
      pre_idx = x->index - 1;
      if(pre_idx > tree_max_index_)
        pre_idx = tree_max_index_;
    }
    else{
      pre_idx = x->index + 1;
      if(pre_idx < tree_min_index_)
        pre_idx = tree_min_index_;
    }
    Eigen::VectorXd cur_state = x->state;
    auto it = node_pool_.lower_bound(string(1, pre_idx));
    double min_dis = 1.0e6;
    ManiPathNodePtr temp, q_near = nullptr;
    // if(!(it != node_pool_.end() && it->second->index == pre_idx)){
    //   std::cout << "pre_idx: " <<  pre_idx << std::endl;
    //   std::cout << calculateValue(pre_idx, Eigen::VectorXi::Zero(manipulator_dof_)) << std::endl;
    //   std::cout << "index: " << it->second->index << std::endl;
    // }
    for(;it != node_pool_.end() && it->second->index == pre_idx; ++it)
    {
      if((dir && it->second->node_state != ManiPathNode::NODE_STATE::IN_TREE) || (!dir && it->second->node_state != ManiPathNode::NODE_STATE::IN_ANTI_TREE))
      {
        continue;
      }
      temp = it->second;
      if(estimateHeuristic(x, temp) < min_dis)
      {
        min_dis = estimateHeuristic(x, temp);
        q_near = temp;
      }
    }
    return q_near;
  }

  ManiPathNodePtr SampleMani::extendNode(ManiPathNodePtr &q_near, ManiPathNodePtr &q_rand, bool flag){
    ManiPathNodePtr q_new = nullptr;
    Eigen::VectorXd dir = q_rand->state - q_near->state;
    double t_total = 0;

    for(int i = min<int>(q_near->index, q_rand->index); i < max<int>(q_near->index, q_rand->index); ++i)
    {
      t_total += t_list_[i];
    }

    Eigen::VectorXd vel_dir = dir / (double)t_total;
    for(int i = 0; i < manipulator_dof_; ++i)
    {
      if(vel_dir(i) > max_joint_vel_ * 0.3)
      {
        vel_dir(i) = max_joint_vel_ * 0.3;
      }
      else if(vel_dir(i) < -max_joint_vel_ *0.3)
      {
        vel_dir(i) = -max_joint_vel_ * 0.3;
      }
    }

    Eigen::VectorXd new_state;
    if(flag)
      new_state = q_near->state + vel_dir * (double)t_list_[q_near->index];
    else
      new_state = q_near->state + vel_dir * (double)t_list_[q_near->index - 1];

    // Eigen::VectorXi new_state_idx = coord2idx(new_state);
    // q_new = node_pool_[q_near->index + 1][findIdxinPool(new_state_idx)];
    int new_idx;
    if(flag)
      new_idx = q_near->index + 1;
    else
      new_idx = q_near->index - 1;

    q_new = initNode(new_idx, new_state);

    if(q_new->node_state == ManiPathNode::NODE_STATE::COLLISION || checkcollision(q_near, q_new)){
      return nullptr;
    }
    return q_new;
  }

  void SampleMani::adjustTree(ManiPathNodePtr &q_new, bool dir){
    if(q_new == nullptr || q_new->index < 1)
    {
      return;
    }
    int next_idx;
    if(dir){
      next_idx = q_new->index + 1;
      if(next_idx > tree_max_index_){
        return;
      }
    }
    else{
      next_idx = q_new->index - 1;
      if(next_idx < tree_min_index_){
        return;
      }
    }
    // int next_check_num = tree_[next_idx].size();
    auto it = node_pool_.lower_bound(string(1, next_idx));
    ManiPathNodePtr q_temp;
    for(;it != node_pool_.end() && it->second->index == next_idx; ++it)
    {
      q_temp = it->second;
      if((dir && q_temp->node_state != ManiPathNode::NODE_STATE::IN_TREE) || (!dir && q_temp->node_state != ManiPathNode::NODE_STATE::IN_ANTI_TREE)){
        continue;
      }

      if(q_temp->g_score > q_new->g_score + estimateHeuristic(q_new, q_temp) && !feasibleCheck(q_new, q_temp))
      {
        if(checkcollision(q_new, q_temp))
        {
          continue;
        }
        // std::cout << "1111111111111111111111111111111111111111" << std::endl;
        // q_temp->g_score = q_new->g_score + estimateHeuristic(q_new, q_temp);
        // q_temp->parent = q_new;
        linkNode(q_new, q_temp);
        // oneShot(q_temp);
      }
    }
  }

  void SampleMani::mergeTrees(const ManiPathNodePtr &q1, const ManiPathNodePtr &q2){

    ManiPathNodePtr q_start, q_end;
    if(q1->node_state == ManiPathNode::NODE_STATE::IN_TREE){
      q_start = q1;
      q_end = q2;
    }else if(q1->node_state == ManiPathNode::NODE_STATE::IN_ANTI_TREE){
      q_start = q2;
      q_end = q1;
    }else{
      return;
    }
    std::vector<ManiPathNodePtr> q_list;
    ManiPathNodePtr q_temp;
    q_temp = q_end;
    while(q_temp != nullptr){
      q_list.push_back(q_temp);
      q_temp->children.clear();
      q_temp = q_temp->parent;
    }

    for(int i = q_list.size() - 1; i > 0; --i){
      q_list[i]->parent = q_list[i-1];
      q_list[i]->node_state = ManiPathNode::NODE_STATE::IN_TREE;
      q_list[i-1]->children.insert(std::make_pair(calculateValue(q_list[i]), q_list[i]));
    }
    linkNode(q_start, q_list[0]);
    q_list[0]->node_state = ManiPathNode::NODE_STATE::IN_TREE;
    this->end_node_ = q_list.back();
  }

  // This function will sort out the whole tree and add the final adjustment process of RRT*.
  void SampleMani::organizeTree(){
    if(node_pool_.size() < 2){
      return;
    }
    std::map<string, ManiPathNodePtr>::iterator it = node_pool_.begin(), temp_it, pre_it;
    ++it;
    int pre_idx;
    ManiPathNodePtr curr_node, pre_node;
    while(it != node_pool_.end()){
      if(it->second->index > max_index_ - 1){
        break;
      }
      temp_it = it;
      ++it;

      curr_node = temp_it->second;
      curr_node->parent = nullptr;
      curr_node->children.clear();
      curr_node->g_score = 1.0e6;
      if(mm_config_->checkManicollision(car_state_list_[curr_node->index], curr_node->state, false)){
        // delete curr_node;
        // node_pool_.erase(temp_it);
        curr_node->node_state = ManiPathNode::NODE_STATE::COLLISION;
        continue;
      }
      pre_idx = temp_it->second->index - 1;
      pre_it = node_pool_.lower_bound(string(1, pre_idx));

      for(; pre_it != node_pool_.end() && pre_it->second->index == pre_idx; ++pre_it){
        pre_node = pre_it->second;
        if(pre_node->g_score + estimateHeuristic(pre_node, curr_node) < curr_node->g_score){
          if(feasibleCheck(pre_node, curr_node) || checkcollision(pre_node, curr_node)){
            continue;
          }
          curr_node->parent = pre_node;
          curr_node->g_score = pre_node->g_score + estimateHeuristic(pre_node, curr_node);
        }
      }

      if(curr_node->parent == nullptr){
        delete curr_node;
        node_pool_.erase(temp_it);
        continue;
      }
      else{
        curr_node->parent->children.insert(std::make_pair(calculateValue(curr_node), curr_node));
        ++tree_count_;
        if(curr_node->index > tree_max_index_){
          tree_max_index_ = curr_node->index;
        }
      }
    }
  }

  // This function will sort out the whole tree but only sort out the existing branches without changing the shape of the tree.
  void SampleMani::organizeTree(ManiPathNodePtr &q){
    if(q->index > max_index_ - 1)
      return;
    // std::cout << "--------------------------0" << std::endl;
    if(q->index > tree_max_index_)
      tree_max_index_ = q->index;
    ++tree_count_;
    if(q->index > max_index_ - 2)
      return;
    std::map<string, ManiPathNodePtr>::iterator it = q->children.begin(), temp_it;
    // std::cout << "--------------------------1" << std::endl;
    while(it != q->children.end()){
      temp_it = it;
      ++it;
      if(feasibleCheck(q, temp_it->second) || 
          mm_config_->checkManicollision(car_state_list_[temp_it->second->index], temp_it->second->state, false) || 
          checkcollision(q, temp_it->second)){
        clearSubTree(temp_it->second);
        q->children.erase(temp_it);
        continue;
      }
      temp_it->second->g_score = q->g_score + estimateHeuristic(q, temp_it->second);
      organizeTree(temp_it->second);
    }
    // std::cout << "--------------------------2" << std::endl;
  }

  void SampleMani::clearSubTree(ManiPathNodePtr &q){
    std::map<string, ManiPathNodePtr>::iterator it;
    for(it = q->children.begin(); it != q->children.end(); ++it){
      clearSubTree(it->second);
    }
    node_pool_.erase(calculateValue(q));
    delete q;
  }

  void SampleMani::oneShot(ManiPathNodePtr &q){
    if(q->index <= 1 || q->index > tree_max_index_){
      return;
    }
    ManiPathNodePtr temp = q->parent->parent;
    ManiPathNodePtr q_check1, q_check2;
    double t_total = t_list_[q->parent->index];
    std::vector<double> t_total_list;
    t_total_list.push_back(0.0);
    t_total_list.push_back(t_total);
    Eigen::VectorXd vel(manipulator_dof_);
    bool is_occ = false;
    std::vector<ManiPathNodePtr> node_list;
    for(;temp != nullptr; temp = temp->parent){
      node_list.clear();
      is_occ = false;
      t_total += t_list_[temp->index];
      t_total_list.push_back(t_total);
      if(feasibleCheck(temp, q))
        continue;
      vel = (temp->state - q->state) / t_total;
      for(int i = 0; i < q->index - temp->index; ++i){
        q_check1 = initNode(q->index - i, q->state + t_total_list.at(i) * vel);
        q_check2 = initNode(q->index - i - 1, q->state + t_total_list.at(i + 1) * vel);
        node_list.push_back(q_check1);
        if(checkcollision(q_check2, q_check1))
        {
          is_occ = true;
          break;
        }
      }
      if(is_occ){
        continue;
      }
      node_list.push_back(temp);
      // std::cout << "shiji: " << node_list.size() << std::endl;
      // std::cout << "lilun: " << q->index - temp->index + 1 << std::endl;

      // TODO: logical error
      // for(int i = 0; i < q->index - temp->index; ++i){
      //   std::cout << "shot!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      //   q_check1 = initNode(q->index - i, coord2idx(q->state + t_total_list.at(i) * vel));
      //   q_check2 = initNode(q->index - i - 1, coord2idx(q->state + t_total_list.at(i + 1) * vel));
      //   // q_check1->parent = q_check2;
      //   linkNode(q_check2, q_check1);
      //   q_check1->node_state = ManiPathNode::NODE_STATE::IN_TREE;
      //   ++tree_count_;
      //   // q_check1->g_score = temp->g_score + estimateHeuristic(temp, q_check1);
      //   // if(q_check2->node_state == ManiPathNode::NODE_STATE::IN_TREE){
      //   //   break;
      //   // }
      //   q_check2->node_state = ManiPathNode::NODE_STATE::IN_TREE;
      // }
      for(int i = node_list.size() - 2; i >= 0; --i){
        q_check1 = node_list[i];
        q_check2 = node_list[i+1];
        if(q_check1->node_state == ManiPathNode::NODE_STATE::IN_TREE){
          if(q_check1->g_score < q_check2->g_score + estimateHeuristic(q_check1, q_check2)){
            continue;
          }
        }
        // std::cout << "shot!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        linkNode(q_check2, q_check1);
        if(q_check1->node_state != ManiPathNode::NODE_STATE::IN_TREE)
          ++tree_count_;
        q_check1->node_state = ManiPathNode::NODE_STATE::IN_TREE;
        
      }
    }
    return;
  }

  void SampleMani::trajShot(ManiPathNodePtr &q){
    if(q->parent){
      trajShot(q->parent);
    }
    oneShot(q);
  }

  void SampleMani::allShot(ManiPathNodePtr &q){
    if(q->index > tree_max_index_)
      return;
    oneShot(q);
    std::vector<ManiPathNodePtr> children_list;
    for(auto it = q->children.begin(); it != q->children.end(); ++it){
      children_list.push_back(it->second);
    }
    int children_num = children_list.size();
    for(int i = 0; i < children_num; ++i){
      if(children_list[i]->parent != q){
        continue;
      }
      allShot(children_list[i]);
    }
  }

  void SampleMani::linkNode(ManiPathNodePtr &parent, ManiPathNodePtr &child){
    // if(parent->index != child->index - 1){
    //   ROS_WARN("[Sample Mani]: link node error! Check the code.");
    //   return;
    // }
    ManiPathNodePtr pre_parent = child->parent;
    if(pre_parent == parent){
      return;
    }
    else if(pre_parent != nullptr){
      pre_parent->children.erase(calculateValue(child));
    }
    child->parent = parent;
    parent->children.insert(std::make_pair(calculateValue(child), child));
    expandGscore(child);
  }

  void SampleMani::expandGscore(ManiPathNodePtr &p){
    if(p->index > max_index_ - 1) return;
    if(p->g_score == p->parent->g_score + estimateHeuristic(p->parent, p))
      return;
    p->g_score = p->parent->g_score + estimateHeuristic(p->parent, p);
    for(auto it = p->children.begin(); it != p->children.end(); ++it){
      expandGscore(it->second);
    }
  }

  bool SampleMani::feasibleCheck(ManiPathNodePtr &x1, ManiPathNodePtr &x2){
    double t_total = 0.0;
    for(int i = min<int>(x1->index, x2->index); i < max<int>(x1->index, x2->index); ++i){
      t_total += t_list_[i];
    }
    Eigen::VectorXd dif = x2->state - x1->state;
    if(fabs(dif(0)) > M_PI){
      dif(0) = 2.0 * M_PI - fabs(dif(0));
    }
    Eigen::VectorXd vel = dif / t_total;
    if(vel.lpNorm<Eigen::Infinity>() > max_joint_vel_){
      return true;
    }
    return false;
  }

  string SampleMani::calculateValue(int &idx, const Eigen::VectorXd &state){
    string ret(1, idx);
    int k;
    for(int i = 0; i < manipulator_dof_; ++i){
      k = round(state(i) * 100.0);
      string s(std::to_string(k));
      ret += s;
    }
    return ret;
  }

  string SampleMani::calculateValue(ManiPathNodePtr &x){
    return calculateValue(x->index, x->state);
  }

  double SampleMani::estimateHeuristic(ManiPathNodePtr &x1, ManiPathNodePtr &x2){
    double t_total = 0.0;
    for(int i = min<int>(x1->index, x2->index); i < max<int>(x1->index, x2->index); ++i){
      t_total += t_list_[i];
    }
    Eigen::VectorXd err = x1->state - x2->state;
    // if(fabs(err(0)) > M_PI){
    //   err(0) = 2.0 * M_PI - fabs(err(0));
    // }
    // for(int i = 0; i < manipulator_dof_; ++i){
    //   err(i) *= (manipulator_dof_ - i + 1) / 2.0;
    // }
    return err.lpNorm<1>() / t_total;
    // return (x1->state - x2->state).lpNorm<1>();
  }

  void SampleMani::init(const std::vector<Eigen::Vector3d> &car_state_list, 
                        const std::vector<Eigen::Vector3d> &car_state_list_check, 
                        const std::vector<double> &t_list){
    this->reset();
    this->car_state_list_ = car_state_list;
    this->car_state_list_check_ = car_state_list_check;
    this->t_list_.clear();
    for(unsigned int i = 0; i < t_list.size(); ++i){
      this->t_list_.push_back(t_list[i] * 3.0);
    }
    // this->t_list_ = t_list * 3.0;
    this->max_index_ = car_state_list.size();
  }

  double SampleMani::calAngleErr(double angle1, double angle2){
    return fabs(angle1 - angle2);
  }

  bool SampleMani::checkcollision(const ManiPathNodePtr& state1, const ManiPathNodePtr& state2){
    ManiPathNodePtr cur_state = state1->index < state2->index ? state1 : state2;
    ManiPathNodePtr next_state = state1->index < state2->index ? state2 : state1;
    if(cur_state->node_state == ManiPathNode::NODE_STATE::COLLISION || next_state->node_state == ManiPathNode::NODE_STATE::COLLISION){
      return true;
    }
    
    Eigen::Matrix4d T_q_now = Eigen::Matrix4d::Zero();
    int index = cur_state->index;
    double tau = t_list_[index];
    
    double dif = (cur_state->state - next_state->state).lpNorm<Eigen::Infinity>();


    if(dif > max_joint_vel_ * tau){
      return true;
    }

    Eigen::Vector3d xt;
    for (int i = 1; i < check_num_; ++i){   
      xt = car_state_list_check_[index * check_num_ + i];
      T_q_now << cos(xt[2]), -sin(xt[2]), 0, xt(0),
                 sin(xt[2]),  cos(xt[2]), 0, xt(1),
                 0,           0,          1, 0,
                 0,           0,          0, 1;
      if(mm_config_->checkManicollision(xt, (cur_state->state + (next_state->state - cur_state->state) * double(i) / double(check_num_)), false)){
        return true;
      }
    }
    return false;
  }

  bool SampleMani::getTraj(std::vector<Eigen::VectorXd> &traj){
    traj.clear();
    if(!have_path_) return false;
    ManiPathNodePtr node = end_node_;
    trajShot(node);
    while(node != nullptr){
      traj.push_back(node->state);
      node = node->parent;
    }
    reverse(traj.begin(), traj.end());
    return true;
  }

  double SampleMani::getCost(){
    if(!have_path_)
    {
      return -1.0;
    }
    ManiPathNodePtr node = end_node_;
    double cost = 0.0;
    while(node != nullptr && node->parent != nullptr)
    {
      cost += estimateHeuristic(node->parent, node);
      node = node->parent;
    }
    return cost;
  }

  void SampleMani::reset(){
    this->max_index_ = 0;
    this->car_state_list_.clear();
    this->car_state_list_check_.clear();
    this->t_list_.clear();
    this->have_path_ = false;
    this->tree_max_index_ = 0;
    this->tree_min_index_ = 1<<20;
    this->tree_count_ = 0;
    this->anti_tree_count_ = 0;
    this->end_node_ = nullptr;

    // memory
    // std::map<string, ManiPathNodePtr>::iterator it, temp_it;
    // it = node_pool_.begin();
    // while(it != node_pool_.end()){
    //   temp_it = it;
    //   ++it;
    //   if(temp_it->second->node_state != ManiPathNode::NODE_STATE::IN_TREE){
    //     delete temp_it->second;
    //     node_pool_.erase(temp_it);
    //   }
    // }


    for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
      delete it->second;
    }
    node_pool_.clear();
  }

  void SampleMani::setParam(ros::NodeHandle& nh, const std::shared_ptr<remani_planner::MMConfig> &mm_config){
    mm_config_ = mm_config;
    manipulator_link_pts_ = mm_config_->getLinkPoint();
    rrt_plan_.reset(new remani_planner::RrtPlanning);
    rrt_plan_->setParam(nh, mm_config);

    nh.param("mm/mobile_base_dof", mobile_base_dof_, -1);
    nh.param("mm/manipulator_dof", manipulator_dof_, -1);
    nh.param("mm/manipulator_thickness", mani_thickness_, -1.0);

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
    nh.param("search/check_num", check_num_, -1);
    nh.param("search/goal_rate", goal_rate_, 0.4);
    nh.param("search/max_loop_num", max_loop_num_, 500);
    nh.param("search/max_mani_search_time", max_mani_search_time_, 0.1);
    nh.param("optimization/self_safe_margin", self_safe_margin_, 0.1);
    nh.param("optimization/safe_margin_mani", safe_margin_mani_, 0.1);
    nh.param("mm/mobile_base_check_radius", mobile_base_check_radius_, 0.1);

    traj_dim_ = mobile_base_dof_ + manipulator_dof_;

    phi_.setIdentity(3, 3);
    T_q_0_ = mm_config_->getTq0();
  }
}  //namespace mani_sample