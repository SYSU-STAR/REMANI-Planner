#include <plan_manage/remani_replan_fsm.h>

namespace remani_planner
{
  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    if(global_plan_) planning_horizen_ = 1.0e3;

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    map_state_pub_ = nh.advertise<std_msgs::Int32>("/map_generator/map_state", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if(try_plan_after_emergency_){
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;
      bool success = planFromGlobalTraj(10);
      // std::cout << "gen new traj 2\n";
      if (success){
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        // ROS_ERROR("Failed to generate new trajectory!!!");
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      
      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      // LocalTrajData *info = &planner_manager_->traj_container_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      bool need_to_plan_next = ((t_cur - info->duration) > time_for_gripper_);
      bool need_to_gripper = (t_cur > info->duration + 0.01);
      t_cur = min(info->duration, t_cur);

      Eigen::VectorXd pos = info->getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);

      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_]; // true: close gripper; false: open
          gripper_cmd_pub_.publish(gripper_cmd);

          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());

          std_msgs::Int32 map_state;
          map_state.data = map_state_;
          // map_state_pub_.publish(map_state);

          // planner_manager_->grid_map_->md_.has_cloud_ = false;

          gripper_flag_ = false;
        }
        
      }else if(t_cur > info->duration - 1e-2 && touch_the_goal){
        
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          /* The navigation task completed */
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");

          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
        
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    // std::cout << "check 3" << std::endl;
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_1_2 = info->duration * 1 / 2;
    double t_2_3 = info->duration * 2 / 3;
    double t_temp;
    bool occ = false;
    // std::cout << "check 4" << std::endl;
    int coll_type;
    for (double t = t_cur; t < info->duration; t += time_step){
      // If t_cur < t_1_2, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_1_2 && t >= t_2_3)
        break;
        
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at relative time %f!", t / info->duration);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at relative time %f!", t / info->duration);
        }
        
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(false)){ // Make a chance
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        // if(planFromLocalTraj(true))
        // {
        //   ROS_INFO("Plan success when detect collision.");
        //   changeFSMExecState(EXEC_TRAJ, "SAFETY");
        //   return;
        // }
        if (t_temp - t_cur < emergency_time_){ // 1.0s of emergency time
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      cout << "Triggered! traget type: " << target_type_ << endl;

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      wpt_id_ = 0;
      planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      return;
    }

    if(msg->pose.position.z < -0.1)
      return;
    cout << "Triggered! traget type: " << target_type_ << endl;
    // trigger_ = true;
    init_state_ = mm_state_pos_;
    end_pt_ = Eigen::VectorXd::Zero(traj_dim_);
    
    if(target_type_ == TARGET_TYPE::MANUAL_TARGET){
      end_pt_(0) = msg->pose.position.x;
      end_pt_(1) = msg->pose.position.y;
      end_yaw_ = tf::getYaw(msg->pose.orientation);
    }else{
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }
    
    planNextWaypoint(end_pt_, end_yaw_);
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    mm_state_vel_(0) = msg->twist.twist.linear.x;
    mm_state_vel_(1) = msg->twist.twist.linear.y;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = 1 ? car_head.dot(mm_state_vel_.head(2)) >= 0 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    for(int i = 0; i < manipulator_dim_; ++i){
      mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      mm_state_vel_(mobile_base_dim_ + i) = msg->velocity[i];
      mm_state_acc_(mobile_base_dim_ + i) = msg->effort[i];
    }
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;
      msg.header.stamp = ros::Time(data->start_time);
      msg.action = msg.ACTION_ADD;
      msg.singul = data->singul_traj[i].singul;
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_.publish(msg);
    }

  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    double t_cur = ros::Time::now().toSec() - info->start_time + replan_trajectory_time_;
    t_cur = min(info->duration, t_cur);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur);
    start_acc_    = info->getAcc(t_cur);
    start_jer_   = info->getJer(t_cur);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
      if (!success)
      {
        return false;
      }
    }

    return true;
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);
    bool local_target_gripper;
    if(reach_horizon){
      local_target_gripper = gripper_state_;
    }else{
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); // global traj is foreward, no need to take singul into account
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time, start_time_dura;
    
    if(have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      start_time_dura = desired_start_time - planner_manager_->traj_container_.singul_traj_data.start_time;
      start_time_dura = min(start_time_dura, planner_manager_->traj_container_.singul_traj_data.duration);
      
      desired_start_pt = planner_manager_->traj_container_.singul_traj_data.getPos(start_time_dura);
      desired_start_vel = planner_manager_->traj_container_.singul_traj_data.getVel(start_time_dura);
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = planner_manager_->traj_container_.singul_traj_data.getSingul(start_time_dura);
      desired_start_acc = planner_manager_->traj_container_.singul_traj_data.getAcc(start_time_dura);
      desired_start_jerk = planner_manager_->traj_container_.singul_traj_data.getJer(start_time_dura);
      desired_start_yaw = atan2(desired_start_singul * desired_start_vel(1), desired_start_singul * desired_start_vel(0));
    }else{
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, have_local_traj_, init_time, opt_time);
    have_new_target_ = false;

    if (plan_success){
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();
      have_local_traj_ = true;

      // vis local traj
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace remani_planner
