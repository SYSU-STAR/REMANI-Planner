// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace remani_planner
{

  MMPlannerManager::MMPlannerManager() {}

  MMPlannerManager::~MMPlannerManager()
  {
    std::cout << "destory manager" << std::endl;
    std_msgs::Bool destory_cmd;
    destory_cmd.data = true;
  }

  void MMPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    visualization_ = vis;
    /* read algorithm parameters */
    
    
    nh.param("mm/manipulator_max_vel", pp_.max_mani_vel_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("search/time_resolution", pp_.polyTraj_piece_time, -1.0);
    double dist_resolution;
    nh.param("search/dist_resolution", dist_resolution, -1.0);
    pp_.polyTraj_piece_time = dist_resolution / pp_.max_vel_;
    nh.param("manager/drone_id", pp_.drone_id, -1);

    nh.param("fsm/planning_horizon", pp_.planning_horizen_, 5.0);
    bool global_plan;
    nh.param("fsm/global_plan", global_plan, false);
    if(global_plan) pp_.planning_horizen_ = 1e3;
    
    nh.param("mm/mobile_base_dof", pp_.mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", pp_.manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", pp_.mobile_base_non_singul_vel_, -1.0);
    destory_cmd_pub_ = nh.advertise<std_msgs::Bool>("/mm_controller_node/destory_cmd", 10);
    pp_.traj_dim_ = pp_.mobile_base_dim_ + pp_.manipulator_dim_;
    total_time_.clear();

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    mm_config_.reset(new MMConfig);
    mm_config_->setParam(nh, grid_map_);

    pp_.max_vel_ = mm_config_->getBaseMaxVel();
    pp_.max_acc_ = mm_config_->getBaseMaxAcc();
    
    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh, grid_map_, mm_config_);
    
  }

  bool MMPlannerManager::computeInitReferenceState(const Eigen::VectorXd &start_pt,
                                                    const Eigen::VectorXd &start_vel,
                                                    const Eigen::VectorXd &start_acc,
                                                    const Eigen::VectorXd &start_jerk,
                                                    const double start_yaw,
                                                    const int start_singul,
                                                    const bool start_gripper,
                                                    const Eigen::VectorXd &local_target_pt,
                                                    const Eigen::VectorXd &local_target_vel,
                                                    const Eigen::VectorXd &local_target_acc,
                                                    const double local_target_yaw,
                                                    const bool local_target_gripper,
                                                    std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container,
                                                    std::vector<int> &singul_container,
                                                    const bool flag_polyInit, 
                                                    const int continous_failures_count)
  {
    static bool flag_first_call = true;
    initMJO_container.clear();
    singul_container.clear();

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit || true){
      // ROS_INFO("get init from search");
      flag_first_call = false;
      /* basic params */
      // std::cout << "computeInit 1\n";
      // Eigen::Matrix<double,4,4> headState, tailState;
      Eigen::MatrixXd headState, tailState;
      headState.resize(pp_.traj_dim_, 4);
      tailState.resize(pp_.traj_dim_, 4);
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;

      // int piece_nums;
      // poly_traj::Trajectory traj;
      
      // constexpr double init_of_init_totaldur = 2.0;

      headState.col(0) = start_pt;
      headState.col(1) = start_vel;
      headState.col(2) = start_acc;
      headState.col(3) = start_jerk;

      tailState.col(0) = local_target_pt;
      tailState.col(1) = local_target_vel;
      tailState.col(2) = local_target_acc;
      tailState.col(3) = Eigen::VectorXd::Zero(pp_.traj_dim_);

      // std::cout << "headState: \n" << headState << std::endl;
      // std::cout << "tailState: \n" << tailState << std::endl;
      // std::cout << "computeInit 2\n";
      /* step 1: A* search and generate init traj */
      vector<vector<Eigen::VectorXd>> simple_path_container;
      vector<Eigen::Vector2d> simple_path;
      vector<vector<double>> yaw_list_container;
      yaw_list_container.clear();
      Eigen::Vector2d init_ctrl;
      init_ctrl.setZero();
      // std::cout << "computeInit 3\n";
      int status = ploy_traj_opt_->astarWithMinTraj(headState, tailState, start_yaw, start_singul, start_gripper,
                                                    local_target_yaw, local_target_gripper, init_ctrl, 
                                                    continous_failures_count,
                                                    simple_path_container, yaw_list_container, 
                                                    initMJO_container, singul_container);
      // std::cout << "computeInit 4\n";
      if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){
        return false;
      }

      // finish get init traj

      // show the init simple_path
      vector<vector<Eigen::Vector2d>> path_view;
      vector<Eigen::Vector2d> display_pts;
      std::vector<Eigen::Vector2d> display_point_set;
      std::vector<Eigen::VectorXd> display_simple_path;
      std::vector<double> display_yaw;
      for(unsigned int i = 0; i < simple_path_container.size(); ++i){
        // std::cout << "computeInit 5 " << i << "\n";
        simple_path.clear();
        for(unsigned int j = 0; j < simple_path_container[i].size(); ++j){
          simple_path.push_back((simple_path_container[i])[j].head(2));
          display_simple_path.push_back((simple_path_container[i])[j]);
          display_yaw.push_back((yaw_list_container[i])[j]);
        }
        path_view.push_back(simple_path);

        Eigen::MatrixXd waypoints;
        Eigen::VectorXd time_list;
        Eigen::MatrixXd ctl_points;
        
        Eigen::Vector2d pts;
        waypoints = initMJO_container[i].getInitConstrainPoints(1);
        time_list = initMJO_container[i].get_T1();
        ctl_points = initMJO_container[i].getInitConstrainPoints(8);
        pts = waypoints.col(0).head(2);
        display_pts.push_back(pts);
        for(unsigned int j = 1; j < waypoints.cols(); ++j)
        {
          pts = waypoints.col(j).head(2);
          display_pts.push_back(pts);
        }
        for (int j = 0; j < ctl_points.cols(); ++j)
          display_point_set.push_back(ctl_points.col(j).head(2));
      }
      visualization_->displayAStarList(path_view, 0);
      visualization_->displayInitWaypoints(display_pts, 0.2, 0);
      visualization_->displayInitPathListDebug(display_point_set, 0.1, 0); // show the init traj for debug
      ploy_traj_opt_->displayFrontEndMesh(display_simple_path, display_yaw);
    }
    /*** case 2: initialize from previous optimal trajectory ***/
    else{ // FIXME check replan
      ROS_INFO("get init from local traj");
      // const double local_target_yaw,
      // std::vector<poly_traj::MinSnapOpt<8>> &initMJO_container,
      // std::vector<int> &singul_container,

      if (traj_container_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_container_.singul_traj_data.start_time;
      double t_to_lc_end = traj_container_.singul_traj_data.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_container_.global_traj.glb_t_of_lc_tgt - traj_container_.global_traj.last_glb_t_of_lc_tgt);
      
      if(t_to_lc_end <= 0){
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but previous trajectories are out of date.");
        return false; // need polyInit
      }
      
      int start_piece_num = traj_container_.singul_traj_data.getPieceIdx(passed_t_on_lctraj);
      int singul_traj_num = traj_container_.singul_traj_data.singul_traj.size() - start_piece_num;
      initMJO_container.resize(singul_traj_num);
      singul_container.resize(singul_traj_num);
      int piece_nums;
      for(int i = 0; i < singul_traj_num; ++i){
        double temp_passed_t, temp_glb_t_remain;
        if(i == 0){
          temp_passed_t = passed_t_on_lctraj;
          traj_container_.singul_traj_data.locatePieceIdx(temp_passed_t);
        }else{
          temp_passed_t = 0;
        }
        if(i == singul_traj_num - 1){
          temp_glb_t_remain = t_to_lc_tgt - t_to_lc_end;
        }else{
          temp_glb_t_remain = 0;
        }
        printf("%d i, duration: %lf, passed_t: %lf, glb_t_remain: %lf", i, traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration, temp_passed_t, temp_glb_t_remain);
        double duration_now = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration - temp_passed_t + temp_glb_t_remain;
        // pp_.polyTraj_piece_time = 1.0;
        std::cout << i << " piece time: " << pp_.polyTraj_piece_time << "\n";
        // piece_nums = ceil(duration_now / pp_.polyTraj_piece_time);
        piece_nums = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
        std::cout << i << " piece num: " << piece_nums << "\n";
        if(piece_nums < 2) piece_nums = 2;
        Eigen::MatrixXd innerPs(pp_.traj_dim_, piece_nums - 1);
        std::cout << "duration: " << duration_now << "\n";
        std::cout << "singul traj duration: " << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration << "\n";
        Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, duration_now / piece_nums);
        double t = piece_dur_vec(0);
        for (int j = 0; j < piece_nums - 1; ++j){
          if (t + temp_passed_t < traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration){
            innerPs.col(j) = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPos(t + temp_passed_t);
            std::cout << "piece: " << start_piece_num + i << " time: " << t + temp_passed_t << " local: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else if (t <= duration_now){
            double glb_t = t + temp_passed_t - traj_container_.singul_traj_data.singul_traj[start_piece_num + i].duration + traj_container_.global_traj.last_glb_t_of_lc_tgt - traj_container_.global_traj.global_start_time;
            innerPs.col(j) = traj_container_.global_traj.traj.getPos(glb_t);
            std::cout << "global: " << innerPs.col(j).head(2).transpose() << "\n";
          }
          else{
            ROS_ERROR("Should not happen! x_x 0x88");
          }

          t += piece_dur_vec(j + 1);
        }

        Eigen::MatrixXd headState, tailState;
        headState.resize(pp_.traj_dim_, 4);
        tailState.resize(pp_.traj_dim_, 4);
        if(i == 0){
          headState.block(0, 0, pp_.traj_dim_, 4) << start_pt, start_vel, start_acc, start_jerk;
        }else{
          headState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(0),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(0);
          // Eigen::VectorXd pos = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPos(1e-3);
          // Eigen::VectorXd vel = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getVel(1e-3);
          // Eigen::VectorXd acc = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getAcc(1e-3);
          // Eigen::VectorXd jer = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(1e-3);
        }

        if(i == singul_traj_num - 1){
          tailState.block(0, 0, pp_.traj_dim_, 4) << local_target_pt, local_target_vel, local_target_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
        }else{
          int PN = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getPieceNum();
          tailState.block(0, 0, pp_.traj_dim_, 4) << traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncPos(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncVel(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncAcc(PN),
                                                     traj_container_.singul_traj_data.singul_traj[start_piece_num + i].traj.getJuncJerk(PN);
        }

        initMJO_container[i].reset(headState, tailState, piece_nums);
        initMJO_container[i].generate(innerPs, piece_dur_vec);
        singul_container[i] = traj_container_.singul_traj_data.singul_traj[start_piece_num + i].singul;
      }
    }

    return true;
  }

  void MMPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::VectorXd &start_pt, const double &start_yaw,
      const Eigen::VectorXd &global_end_pt, const double global_end_yaw, 
      Eigen::VectorXd &local_target_pos, Eigen::VectorXd &local_target_vel,Eigen::VectorXd &local_target_acc, bool &reach_horizon)
  {
    reach_horizon = true;
    double t;

    traj_container_.global_traj.last_glb_t_of_lc_tgt = traj_container_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    for (t = traj_container_.global_traj.glb_t_of_lc_tgt;
         t < (traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration);
         t += t_step){
      Eigen::VectorXd pos_t = traj_container_.global_traj.traj.getPos(t - traj_container_.global_traj.global_start_time);

      double dist = ((pos_t - start_pt).head(pp_.mobile_base_dim_)).norm();
      if (dist >= planning_horizen){
        bool occ;
        double yaw;
        Eigen::VectorXd vel;
        if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){
          yaw = global_end_yaw;
        }else{
          vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
          yaw = atan2(vel(1), vel(0));
        }
        occ = mm_config_->checkcollision(Eigen::Vector3d(pos_t(0), pos_t(1), yaw), pos_t.tail(pp_.manipulator_dim_), false);
        if(occ) continue;
        local_target_pos = pos_t;
        traj_container_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_container_.global_traj.global_start_time) >= traj_container_.global_traj.duration){ // Last global point
      local_target_pos = global_end_pt;
      traj_container_.global_traj.glb_t_of_lc_tgt = traj_container_.global_traj.global_start_time + traj_container_.global_traj.duration;
      reach_horizon = false;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_)){
      local_target_vel = Eigen::VectorXd::Zero(pp_.traj_dim_);
      local_target_vel.head(2) = pp_.mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(global_end_yaw), sin(global_end_yaw));
      local_target_acc = Eigen::VectorXd::Zero(pp_.traj_dim_);
    }else{
      local_target_vel = traj_container_.global_traj.traj.getVel(t - traj_container_.global_traj.global_start_time);
      local_target_acc = traj_container_.global_traj.traj.getAcc(t - traj_container_.global_traj.global_start_time);
    }
  }

  bool MMPlannerManager::reboundReplan(
      const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, 
      const Eigen::VectorXd &start_acc,const Eigen::VectorXd &start_jerk,
      const double start_yaw, const int start_singul, const bool start_gripper, const double trajectory_start_time, 
      const Eigen::VectorXd &local_target_pt, const Eigen::VectorXd &local_target_vel,
      const Eigen::VectorXd &local_target_acc, double local_target_yaw, const bool local_target_gripper,
      const bool flag_polyInit, const bool flag_randomPolyTraj,
      const bool have_local_traj, double &init_time, double &opt_time)
  {
    static int count = 0;

    printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    std::vector<poly_traj::MinSnapOpt<8>> initMJO_container;
    std::vector<int> singul_container;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc, start_jerk, start_yaw, start_singul, start_gripper,
                                   local_target_pt, local_target_vel, local_target_acc, local_target_yaw, local_target_gripper,
                                   initMJO_container, singul_container, 
                                   flag_polyInit, continous_failures_count_)){return false;}

    Eigen::VectorXd init_len(7);
    double init_dura = 0.0;
    init_len.setZero();
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){
      Eigen::MatrixXd cst_pts = initMJO_container[i].getInitConstrainPoints(1);
      Eigen::VectorXd time_list = initMJO_container[i].get_T1();
      Eigen::VectorXd pos1 = cst_pts.col(0), pos2;
      init_dura += time_list.lpNorm<1>();
      for(unsigned int j = 1; j < cst_pts.cols(); ++j){
        pos2 = cst_pts.col(j);
        init_len(0) += (pos2 - pos1).head(2).norm();
        init_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();
        pos1 = pos2;
      }
    }
    


    if(initMJO_container.size() != singul_container.size()){
      ROS_ERROR("initMJO_container size = %d != singul_container size = %d", (int)initMJO_container.size(), (int)singul_container.size());
    }
    ploy_traj_opt_->clear_resize_Cps_container(initMJO_container.size());
    std::vector<Eigen::Vector2d> disp_point_set;
    std::vector<Eigen::MatrixXd> iniStates_container;
    std::vector<Eigen::MatrixXd> finStates_container;
    std::vector<Eigen::MatrixXd> initInnerPts_container;
    std::vector<Eigen::VectorXd> initT_container;
    Eigen::MatrixXd cstr_pts;
    for(unsigned int i = 0; i < initMJO_container.size(); ++i){
      cstr_pts = initMJO_container[i].getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      ploy_traj_opt_->setControlPoints(i, cstr_pts);

      poly_traj::Trajectory<7> initTraj = initMJO_container[i].getTraj(singul_container[i]);

      initT_container.push_back(initTraj.getDurations());

      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, pp_.traj_dim_, PN - 1);
      initInnerPts_container.push_back(innerPts);

      Eigen::MatrixXd headState, tailState;
      headState.resize(pp_.traj_dim_, 4);
      tailState.resize(pp_.traj_dim_, 4);
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0),initTraj.getJuncJerk(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN),initTraj.getJuncJerk(PN);
      iniStates_container.push_back(headState);
      finStates_container.push_back(tailState);
      for (int j = 0; j < cstr_pts.cols(); ++j)
        disp_point_set.push_back(cstr_pts.col(j).head(2));
    }
    visualization_->displayInitCtrlPts(disp_point_set, 0.06, 0);
    
    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();
    
    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    
    
    Eigen::MatrixXd opt_waypoints;
    Eigen::VectorXd time_list;
    vector<vector<Eigen::Vector2d>> his_trajs;
    
    
    std::vector<Eigen::VectorXd> optT_container, optEECps_container;
    std::vector<Eigen::MatrixXd> optWps_container;
    std::vector<Eigen::MatrixXd> optCps_container;
    
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(iniStates_container, finStates_container,
                                                            initInnerPts_container, initT_container, singul_container,
                                                            optCps_container, optWps_container, optT_container, optEECps_container);
    t_opt = ros::Time::now() - t_start;

    // calculate data
    double snap_cost = 0.0, traj_dura = 0.0;
    Eigen::VectorXd traj_len(7);
    traj_len.setZero();
    double traj_time;
    Eigen::VectorXd pos1, pos2;
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      snap_cost += (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTrajJerkCost();
      traj_time = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getTotalDuration();
      traj_dura += traj_time;

      pos1 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(0.0);
      for(double j = 0.0; j < traj_time + 1.0e-3; j += 0.01){
        pos2 = (*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]).getPos(j);
        traj_len(0) += (pos2 - pos1).head(2).norm();
        traj_len.tail(6) += ((pos2 - pos1).tail(6)).cwiseAbs();
        pos1 = pos2;
      }
    }
    
    if(!flag_success){
      for(unsigned int i = 0; i < optCps_container.size(); ++i){
        visualization_->displayFailedList(optCps_container[i], i);
      }
      continous_failures_count_++;
      return false;
    }
    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    init_time = t_init.toSec() * 1000;
    opt_time = t_opt.toSec() * 1000;
    cout << "\033[34mtotal time: " << (t_init + t_opt).toSec() * 1000
         << ", init: " << t_init.toSec() * 1000
         << ", optimize: " << t_opt.toSec() * 1000
         << ", avg_time: " << sum_time / count_success * 1000.0
         << ", count_success: " << count_success << "\033[0m"<< endl;
    average_plan_time_ = sum_time / count_success;

    double traj_start_time;
    if(have_local_traj){
      double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      if (delta_replan_time > 0) ros::Duration(delta_replan_time).sleep();
      traj_start_time = trajectory_start_time;
    }
    else{
      traj_start_time = ros::Time::now().toSec();
    }

    traj_container_.singul_traj_data.clearSingulTraj();
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      traj_container_.singul_traj_data.addSingulTraj((*ploy_traj_opt_->getMinSnapOptContainerPtr())[i].getTraj(singul_container[i]), traj_start_time);
      traj_start_time = traj_container_.singul_traj_data.singul_traj.back().end_time;
    }
    visualization_->displayOptimalCtrlPts(optCps_container, 0);

    vector<Eigen::Vector2d> display_pts;
    Eigen::Vector2d pts;
    for(unsigned int i = 0; i < optWps_container.size(); ++i){
      display_pts.clear();
      opt_waypoints = optWps_container[i];
      time_list = optT_container[i];
      pts = opt_waypoints.col(0).head(2);
      display_pts.push_back(pts);
      for(unsigned int j = 1; j < opt_waypoints.cols(); ++j)
      {
        pts = opt_waypoints.col(j).head(2);
        display_pts.push_back(pts);
      }
      visualization_->displayOptWaypoints(display_pts, 0.2, i);
    }
    
    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool MMPlannerManager::EmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw,  const int singul){
    auto ZERO = Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);
    tailState.resize(pp_.traj_dim_, 4);
    headState.block(0, 0, pp_.traj_dim_, 4) << stop_pos, ZERO, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinSnapOpt<8> stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_container_.singul_traj_data.clearSingulTraj();
    traj_container_.singul_traj_data.addSingulTraj(stopMJO.getTraj(singul), ros::Time::now().toSec());

    return true;
  }

  // FIXME singul
  bool MMPlannerManager::planGlobalTrajWaypoints(
      const Eigen::VectorXd &start_pos, const double start_yaw, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
      const std::vector<Eigen::VectorXd> &waypoints, const double end_yaw, const Eigen::VectorXd &end_vel, const Eigen::VectorXd &end_acc)
  {
    int start_singul = 1;
    poly_traj::MinSnapOpt<8> globalMJO;
    Eigen::MatrixXd headState, tailState;
    headState.resize(pp_.traj_dim_, 4);
    tailState.resize(pp_.traj_dim_, 4);
    headState << start_pos, start_vel, start_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    tailState << waypoints.back(), end_vel, end_acc, Eigen::VectorXd::Zero(pp_.traj_dim_);
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(pp_.traj_dim_, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());
    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());
    int try_num = 0;
    do
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).head(pp_.mobile_base_dim_).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).head(pp_.mobile_base_dim_).norm() / des_vel;
        for(int j = 0; j < pp_.manipulator_dim_; ++j){
          double t_temp;
          if(i == 0){
            t_temp = fabs((waypoints[0] - start_pos)[pp_.mobile_base_dim_ + j]) / pp_.max_mani_vel_;
          }else{
            t_temp = fabs((waypoints[i] - waypoints[i - 1])[pp_.mobile_base_dim_ + j]) / pp_.max_mani_vel_;
          }
          time_vec(i) = max(time_vec(i), t_temp);
        }
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj(start_singul).getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;
      des_vel /= 1.5;
      try_num++;
    } while (globalMJO.getTraj(start_singul).getMaxVelRate() > pp_.max_vel_ && try_num <= 5);
    auto time_now = ros::Time::now();
    traj_container_.setGlobalTraj(globalMJO.getTraj(start_singul), time_now.toSec());

    return true;
  }
} // namespace remani_planner
