#include "optimizer/poly_traj_optimizer.hpp"

namespace remani_planner
{
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh, const std::shared_ptr<GridMap> &map, const std::shared_ptr<MMConfig> &mm_config){
    grid_map_ = map;
    map_resolution_ = grid_map_->getResolution();

    mm_config_ = mm_config;
    max_vel_ = mm_config_->getBaseMaxVel();
    max_acc_ = mm_config_->getBaseMaxAcc();
    manipulator_config_ = mm_config_->getManiConfig();
    T_q_0_ = mm_config_->getTq0();
    manipulator_link_pts_ = mm_config_->getLinkPoint();

    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_base_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    wei_mani_obs_ = wei_obs_ / 5.0;
    nh.param("optimization/weight_manipulator_self", wei_mani_self_, -1.0);
    nh.param("optimization/weight_manipulator_feasibility", wei_mani_feas_, -1.0);

    nh.param("optimization/dense_sample_resolution", dense_sample_resolution_, -1);

    nh.param("optimization/safe_margin", safe_margin_, 0.1);
    nh.param("optimization/safe_margin_mani", safe_margin_mani_, 0.1);
    nh.param("optimization/self_safe_margin", self_safe_margin_, 0.1);
    nh.param("optimization/ground_safe_dis", ground_safe_dis_, 0.1);
    nh.param("optimization/ground_safe_margin", ground_safe_margin_, 0.1);
    nh.param("optimization/mobile_base_opt_gear", opt_gear_, true);

    nh.param("mm/mobile_base_dof", mobile_base_dof_, -1);
    nh.param("mm/mobile_base_non_singul_vel", non_singul_v_, -1.0);
    nh.param("mm/mobile_base_max_wheel_omega", max_wheel_omega_, -1.0);
    nh.param("mm/mobile_base_max_wheel_alpha", max_wheel_alpha_, -1.0);
    nh.param("mm/mobile_base_wheel_base", mobile_base_wheel_base_, -1.0);
    nh.param("mm/mobile_base_wheel_radius", mobile_base_wheel_radius_, -1.0);
    nh.param("mm/mobile_base_length", mobile_base_length_, -1.0);
    nh.param("mm/mobile_base_width", mobile_base_width_, -1.0);
    nh.param("mm/mobile_base_height", mobile_base_height_, -1.0);
    nh.param("mm/mobile_base_check_radius", mobile_base_check_radius_, -1.0);

    nh.param("mm/manipulator_dof", manipulator_dof_, -1);
    nh.param("mm/manipulator_thickness", manipulator_thickness_, -1.0);
    
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

    firs_plot_ = true;

    B_h_ << 0.0, -1.0,
            1.0,  0.0;
    C_h_ = Eigen::MatrixXd::Zero(3, 4);
    C_h_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    kino_a_star_.reset(new KinoAstar);
    kino_a_star_->setParam(nh, map, mm_config);
    // kino_a_star_->init();

    traj_dim_ = mobile_base_dof_ + manipulator_dof_;
    traj_pt_pub_ = nh.advertise<visualization_msgs::Marker>("traj_pt_vis", 10);
    traj_init_pt_pub_ = nh.advertise<visualization_msgs::Marker>("traj_init_pt_vis", 10);
    front_end_mm_mesh_vis_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("front_end_mm_mesh_vis", 50);
    back_end_mm_mesh_vis_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("back_end_mm_mesh_vis", 50);
  }

  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(const std::vector<Eigen::MatrixXd> &iniState_container, 
                                                    const std::vector<Eigen::MatrixXd> &finState_container,
                                                    const std::vector<Eigen::MatrixXd> &initInnerPts_container, 
                                                    const std::vector<Eigen::VectorXd> &initT_container, 
                                                    const std::vector<int> &singul_container,
                                                    std::vector<Eigen::MatrixXd> &optCps_container, 
                                                    std::vector<Eigen::MatrixXd> &optWps_container, 
                                                    std::vector<Eigen::VectorXd> &optT_container,
                                                    std::vector<Eigen::VectorXd> &optEECps_container){
    traj_num_ = initInnerPts_container.size();
    iniState_container_ = iniState_container;
    finState_container_ = finState_container;
    singul_container_ = singul_container;
    variable_num_ = 0;
    SnapOpt_container_.clear();
    piece_num_container_.clear();
    SnapOpt_container_.resize(traj_num_);
    piece_num_container_.resize(traj_num_);
    if (traj_num_ != (int)initT_container.size()){
      ROS_ERROR("traj_num_ != initT_container.size()");
      return false;
    }

    int piece_num;
    int piece_num_all = 0;
    for(int i = 0; i < traj_num_; i++){
      //check
      if(initInnerPts_container[i].cols()==0){
        ROS_ERROR("There is only a piece?");
        return false;
      }
      piece_num = initInnerPts_container[i].cols() + 1;
      piece_num_container_[i] = piece_num;
      piece_num_all += piece_num;

      //reset the start end max_vel_
      if(iniState_container_[i].col(1).head(2).norm() >= max_vel_){
        iniState_container_[i].col(1).head(2) = iniState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }
      if(finState_container_[i].col(1).head(2).norm() >= max_vel_){
        finState_container_[i].col(1).head(2) = finState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }

      if(iniState_container_[i].col(2).head(2).norm() >= max_acc_){
        iniState_container_[i].col(2).head(2) = iniState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }
      if(finState_container_[i].col(2).head(2).norm() >= max_acc_){
        finState_container_[i].col(2).head(2) = finState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }
      
      SnapOpt_container_[i].reset(piece_num);
      variable_num_ += traj_dim_ * (piece_num - 1); // inner pt
      variable_num_ += piece_num; // time allocation
    }
    
    variable_num_ += traj_dim_ * (traj_num_ - 1); // gear pos
    variable_num_ += 1 * (traj_num_ - 1); // gear angle

    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    // inner pt
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, initInnerPts_container[i].data(), initInnerPts_container[i].size() * sizeof(x[0]));
      offset += initInnerPts_container[i].size();
    }
    // time allocation
    Eigen::VectorXd initT;
    int offset_temp = 0;
    initT.resize(piece_num_all);
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, initT_container[i].data(), initT_container[i].size() * sizeof(x[0]));
      memcpy(initT.data() + offset_temp, initT_container[i].data(), initT_container[i].size() * sizeof(initT[0]));
      offset += initT_container[i].size();
      offset_temp += initT_container[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset - piece_num_all, piece_num_all);
    RealT2VirtualT(initT, Vt);
    // gear pos
    for(int i = 0; i < traj_num_ - 1; i++){
      memcpy(x.data() + offset, finState_container_[i].col(0).data(), traj_dim_ * sizeof(x[0]));
      offset += traj_dim_;
    }
    // gear angle
    Eigen::Map<Eigen::VectorXd> angles(x.data() + offset, traj_num_ - 1);
    for(int i = 0; i < traj_num_ - 1; i++){
      Eigen::Vector2d gearv = finState_container_[i].col(1).head(2);
      angles[i] = std::atan2(gearv[1], gearv[0]);
    }
    
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1e-3;
    lbfgs_params.max_iterations = 1000;

    double final_cost;
    
    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;

    /* ---------- optimize ---------- */
    // ros::Time t1 = ros::Time::now();
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);

    // printf("\033[32miter = %d, time = %5.3f(ms), \n\033[0m", iter_num_, (ros::Time::now() - t1).toSec() * 1000.0);
    if((result != lbfgs::LBFGS_CONVERGENCE)&&(result != lbfgs::LBFGS_STOP)&&(result != lbfgs::LBFGSERR_MAXIMUMLINESEARCH)){
      ROS_ERROR("The optimization result is : %s", lbfgs::lbfgs_strerror(result));      
    }else if(result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      ROS_WARN("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    }else{
      ROS_INFO("The optimization result is : %s", lbfgs::lbfgs_strerror(result));      
    }

    // test collision
    SingulTrajData singul_traj_data;
    double traj_start_time = 0;
    singul_traj_data.clearSingulTraj();
    for(unsigned int i = 0; i < singul_container.size(); ++i){
      singul_traj_data.addSingulTraj(SnapOpt_container_[i].getTraj(singul_container[i]), traj_start_time);
      traj_start_time = singul_traj_data.singul_traj.back().end_time;
    }
    bool good_traj = true;
    good_traj = IsTrajSafe(singul_traj_data);
    if(result == lbfgs::LBFGSERR_INVALID_FUNCVAL){
      return false;
    }

    optCps_container.clear();
    optWps_container.clear();
    optT_container.clear();
    optEECps_container.clear();
    for(int i = 0; i < traj_num_; i++){
      optCps_container.push_back(cps_container_[i].points);
      optWps_container.push_back(SnapOpt_container_[i].getInitConstrainPoints(1));
      optT_container.push_back(SnapOpt_container_[i].get_T1());
    }

    return good_traj;
  }

  void PolyTrajOptimizer::displayBackEndMesh(const SingulTrajData &traj_data, bool init, bool gripper_close){
    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::MarkerArray marker_array, marker_array_all, marker_array_i;
    visualization_msgs::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);

    Eigen::VectorXd pos, vel;
    int i = 0;
    double yaw;
    double t_step = 0.5 / max_vel_;
    double duration = traj_data.duration;
    for(double t = 0; t < duration - 1e-3; t += t_step, ++i){
      pos = traj_data.getPos(t);
      vel = traj_data.getVel(t);
      yaw = traj_data.getCarAngle(t);

      car_state.head(2) = pos.head(2);
      car_state(2) = yaw;
      joint_state = pos.tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_back_end", i, 0.17, car_state, joint_state, gripper_close);
      marker_array_all.markers.insert(marker_array_all.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
      back_end_mm_mesh_vis_pub_.publish(marker_array_all);
      ros::Duration(0.05).sleep();
    }
  }

  bool PolyTrajOptimizer::smoothedL1(const double &x, const double &mu, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);

      return true;
    }
  }

  bool PolyTrajOptimizer::smoothedMax3(const double &x, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }else{
      f = x * x * x;
      df = 3.0 * x * x;
      return true;
    }
  }

  bool PolyTrajOptimizer::smoothedLog(const double &x, const double &mu, double &l, double &grad){
    if (x <= -mu){
      l = 0.0;
      grad = 0.0;
      return false;
    }else if (x <= 0){
      double e1 = x + mu;
      double e1_2 = e1 * e1;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * e1_2 * e1 * (mu - x) * mu_4;
      grad = e1_2 * (mu - 2 * x) * mu_4;
      return true;
    }else if(x <= mu){
      double e2 = x - mu;
      double e2_2 = e2 * e2;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * (x + mu) * e2_2 * e2 * mu_4 + 1.0;
      grad = e2_2 * (mu + 2 * x) * mu_4;
      return true;
    }else{
      l = 1.0;
      grad = 0.0;
      return true;
    }
  }

  bool PolyTrajOptimizer::IsNotFeasibie(const SingulTrajData &traj_data, double t){
    Eigen::VectorXd vel, acc, jer;
    double pen;
    double feas_tol_percent_ = 0.05;

    vel = traj_data.getVel(t).head(2);
    acc = traj_data.getAcc(t).head(2);
    jer = traj_data.getJer(t).head(2);
    int singul = traj_data.getSingul(t);

    double aTv = acc.transpose() * vel;
    double aTBv = acc.transpose() * B_h_ * vel;
    double jTBv = jer.transpose() * B_h_ * vel;
    double v_norm = vel.norm();
    double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
    double vTv_inv2 = vTv_inv * vTv_inv;

    // Check minimum speed
    // pen =  non_singul_v_ *  non_singul_v_ * (1.0 - feas_tol_percent_) * (1.0 - feas_tol_percent_) - vel.squaredNorm();
    // if(pen > 0.0){
    //   ROS_WARN("%f min vel is not feasible at relative time %f! opt failed", vel.norm(), t);
    //   return true;
    // }

    double omega = aTBv * vTv_inv;
    // Check maximum rotation speed of the left wheel
    double wheel_omega_left = (2.0 * singul * v_norm - mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max left wheel omega is not feasible at relative time %f! opt failed", wheel_omega_left, t );
      return true;
    }

    // Check maximum rotation speed of the right wheel
    double wheel_omega_right = (2.0 * singul * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max right wheel omega is not feasible at relative time %f! opt failed", wheel_omega_right, t );
      return true;
    }

    // Check maximum rotational acceleration of the left wheel
    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
    double wheel_alpha_left = (2.0 * singul * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max left wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_left, t );
      return true;
    }

    // Check maximum acceleration of the right wheel
    double wheel_alpha_right = (2.0 * singul * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max right wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_right, t );
      return true;
    }

    // Manipulator
    vel = traj_data.getVel(t).tail(manipulator_dof_);
    acc = traj_data.getAcc(t).tail(manipulator_dof_);
    if(vel.lpNorm<Eigen::Infinity>() - max_joint_vel_ * (1.0 + feas_tol_percent_) > 0){
      ROS_WARN("mani vel %f is not feasible at relative time %f! opt failed", vel.lpNorm<Eigen::Infinity>(), t);
      return true;
    }

    if(acc.lpNorm<Eigen::Infinity>() - max_joint_acc_ * (1.0 + feas_tol_percent_) > 0){
      ROS_WARN("mani acc %f is not feasible at relative time %f! opt failed", acc.lpNorm<Eigen::Infinity>(), t);
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::IsTrajSafe(const SingulTrajData &traj_data){
    double dt = 0.01;
    double T_all = traj_data.duration;
    int i_end = floor(T_all / dt); // check all
    double t = 0.05;  //skip the start point
    // double t = 0.00;  //do not skip the start point
    int coll_type;
    for (int i = 5; i < i_end; i++){
      if(checkCollision(traj_data, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at time %f!", t);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at time %f!", t);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at time %f!", t);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at time %f!", t);
        }
        // ROS_WARN("mm (%d) collision at time %f! opt failed", coll_type, t);
        return false;
      }

      if(IsNotFeasibie(traj_data, t)){
        return false;
      }
      
      t += dt;
    }
    return true;
  }

  bool PolyTrajOptimizer::checkCollision(const SingulTrajData &traj, double t, int &coll_type)
  {
    Eigen::VectorXd pos = traj.getPos(t);
    Eigen::VectorXd vel = traj.getVel(t);
    double yaw = traj.getCarAngle(t);
    return mm_config_->checkcollision(Eigen::Vector3d(pos(0), pos(1), yaw), pos.tail(manipulator_dof_), false, coll_type);
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);
    double total_smoo_cost = 0, total_time_cost = 0;
    Eigen::VectorXd obs_feas_costs(6), total_obs_feas_costs(6); // car-obs mani-obs mm-mm car_feasible mani_feasible quratic_variance
    obs_feas_costs.setZero();
    total_obs_feas_costs.setZero();
    int offset = 0;
    
    // inner point
    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, opt->traj_dim_, opt->piece_num_container_[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(g.data() + offset, opt->traj_dim_, opt->piece_num_container_[trajid] - 1);
      offset += opt->traj_dim_ * (opt->piece_num_container_[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }

    std::vector<Eigen::VectorXd> T_container; // real time
    std::vector<Eigen::VectorXd> gradT_container;
    std::vector<Eigen::Map<const Eigen::VectorXd>> t_container; // virtual time
    std::vector<Eigen::Map<Eigen::VectorXd>> gradt_container;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      int piece_num = opt->piece_num_container_[trajid];
      Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, piece_num);
      Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, piece_num); 
      gradt.setZero();
      t_container.push_back(t);
      gradt_container.push_back(gradt);
      
      Eigen::VectorXd T(piece_num);
      opt->VirtualT2RealT(t, T);
      Eigen::VectorXd gradT(piece_num); 
      gradT.setZero();
      T_container.push_back(T);
      gradT_container.push_back(gradT);

      offset += piece_num;
    }

    // Ini/Fin Gear Pos
    std::vector<Eigen::Map<const Eigen::MatrixXd>> Gear_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradGear_container;
    for(int trajid = 0; trajid < opt->traj_num_ - 1; trajid++){
      Eigen::Map<const Eigen::MatrixXd> Gear(x.data() + offset, opt->traj_dim_, 1);
      Eigen::Map<Eigen::MatrixXd>gradGear(g.data() + offset, opt->traj_dim_, 1);
      offset += opt->traj_dim_;
      gradGear.setZero();
      // std::cout << "Gear: " << Gear.transpose() << "\n";
      Gear_container.push_back(Gear);
      gradGear_container.push_back(gradGear);
    }

    // gear angle
    Eigen::Map<const Eigen::VectorXd> Angles(x.data() + offset, opt->traj_num_ - 1);
    Eigen::Map<Eigen::VectorXd>gradAngles(g.data() + offset, opt->traj_num_ - 1);
    gradAngles.setZero();

    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::MatrixXd IniS, FinS;
      IniS = opt->iniState_container_[trajid];
      FinS = opt->finState_container_[trajid];

      if(trajid > 0){
        double theta = Angles[trajid - 1];
        IniS.col(0) = Gear_container[trajid - 1];
        IniS.col(1).head(2) = Eigen::Vector2d(-opt->non_singul_v_ * cos(theta), -opt->non_singul_v_ * sin(theta)); 
      }
      if(trajid < opt->traj_num_ - 1){
        double theta = Angles[trajid];
        FinS.col(0) = Gear_container[trajid];
        FinS.col(1).head(2) = Eigen::Vector2d(opt->non_singul_v_ * cos(theta), opt->non_singul_v_ * sin(theta));
      }
      
      opt->SnapOpt_container_[trajid].generate(P_container[trajid], T_container[trajid], IniS, FinS);
      double smoo_cost = 0;
      opt->initAndGetSmoothnessGradCost2PT(trajid, gradT_container[trajid], smoo_cost); // Smoothness cost
      obs_feas_costs.setZero();
      opt->addPVAJGradCost2CT(trajid, gradT_container[trajid], obs_feas_costs, opt->cps_num_prePiece_); // Time int cost
      
      //Get gradT gradC
      total_smoo_cost += smoo_cost;
      total_obs_feas_costs += obs_feas_costs;
    }

    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      double time_cost = 0.0;
      Eigen::MatrixXd gradIni, gradFin;
      //waypoint
      opt->SnapOpt_container_[trajid].getGrad2TP(gradT_container[trajid], gradP_container[trajid], gradIni, gradFin);
      if(opt->opt_gear_){
        if(trajid > 0){
          double theta = Angles[trajid - 1];
          gradGear_container[trajid - 1] += gradIni.col(0);
          gradAngles[trajid - 1] += gradIni.topRows(2).col(1).transpose() * Eigen::Vector2d(opt->non_singul_v_ * sin(theta), -opt->non_singul_v_*cos(theta));
        }
        if(trajid < opt->traj_num_ - 1){
          double theta = Angles[trajid];
          gradGear_container[trajid] += gradFin.col(0);
          gradAngles[trajid] += gradFin.topRows(2).col(1).transpose() * Eigen::Vector2d(-opt->non_singul_v_ * sin(theta), opt->non_singul_v_*cos(theta));
        }
      }
      opt->VirtualTGradCost(T_container[trajid], t_container[trajid], gradT_container[trajid], gradt_container[trajid], time_cost);
      total_time_cost += time_cost;
    }
    double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost;
    opt->iter_num_ += 1;

    return costall;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data,const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(const int trajid, EIGENVEC &gdT, double &cost)
  {
    SnapOpt_container_[trajid].initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(const int trajid, EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K){
    costs.setZero();
    int N = gdT.size();
    Eigen::VectorXd pos(traj_dim_), pos_next(traj_dim_), vel(traj_dim_), acc(traj_dim_), jer(traj_dim_), snap(traj_dim_);
    Eigen::VectorXd dense_pos(traj_dim_), dense_vel(traj_dim_), dense_acc(traj_dim_), dense_jer(traj_dim_), dense_sna(traj_dim_);
    Eigen::VectorXd gradp(traj_dim_), gradv(traj_dim_), grada(traj_dim_), gradj(traj_dim_), gradp_traj_piece(traj_dim_);
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4; 
    Eigen::Matrix<double, 8, 1> Densebeta0, Densebeta1, Densebeta2, Densebeta3, Densebeta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    Eigen::MatrixXd gradViolaVc_dense, gradViolaAc_dense, gradViolaJc_dense;
    gradViolaPc.resize(8, traj_dim_);
    gradViolaVc.resize(8, traj_dim_);
    gradViolaAc.resize(8, traj_dim_);
    gradViolaJc.resize(8, traj_dim_);
    gradViolaVc_dense.resize(8, traj_dim_);
    gradViolaAc_dense.resize(8, traj_dim_);
    gradViolaJc_dense.resize(8, traj_dim_);
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaVt_dense, gradViolaAt_dense, gradViolaJt_dense;

    double omg, dense_omg;
    int i_dp = 0;

    for (int i = 0; i < N; ++i){
      
      Eigen::MatrixXd c;
      c.resize(8, traj_dim_);
      c = SnapOpt_container_[trajid].get_b().block(i * 8, 0, 8, traj_dim_);
      
      step = SnapOpt_container_[trajid].get_T1()(i) / K;
      s1 = 0.0;
      gradp_traj_piece.setZero();

      
      for (int j = 0; j <= K; ++j){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;
        cps_container_[trajid].points.col(i_dp) = pos;
        // collision
        gradp.setZero();
        gradv.setZero();
        double cost_car = 0;
        double cost_mani = 0;
        double cost_self = 0;
        
        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv, cost_car, cost_mani, cost_self))
        {
          gradp_traj_piece += gradp;
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * ((cost_car + cost_mani + cost_self) / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          costs(0) += omg * step * cost_car;
          costs(1) += omg * step * cost_mani;
          costs(2) += omg * step * cost_self;
        }
        
        // car feasibility
        Eigen::Vector2d gradv_2d, grada_2d, gradj_2d;
        gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
        double cost_mm_feasible = 0.0;
        if(feasibilityGradCostCar(vel.head(2), acc.head(2), jer.head(2), trajid,
                                  gradv_2d, grada_2d, gradj_2d,
                                  cost_mm_feasible, false)){
          gradv.setZero(); grada.setZero(); gradj.setZero();
          gradv.head(2) = gradv_2d; grada.head(2) = grada_2d; gradj.head(2) = gradj_2d;

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (cost_mm_feasible / K + step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaJc;
          gdT(i) += omg * (step * gradViolaJt);

          costs(3) += omg * step * cost_mm_feasible;
        }
        
        // dense sampling
        double min_dense_vel = 0.1; 
        double pen_min_vel = min_dense_vel * min_dense_vel - vel.head(2).squaredNorm();
        double pen_min_vel_log, grad_pen_min_vel_log;
        if(smoothedLog(pen_min_vel, 0.001, pen_min_vel_log, grad_pen_min_vel_log)){
          double special_step = step / dense_sample_resolution_;
          double special_s1 = s1;
          int disQuantity;
          double dense_alpha;
          Eigen::VectorXd gradv_dense(traj_dim_), grada_dense(traj_dim_), gradj_dense(traj_dim_);
          if(j == 0){
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 1.0 / K / dense_sample_resolution_;
          }else if(j == K){
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }else{
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }

          for(int l = 0; l <= disQuantity; l++){
            s2 = special_s1 * special_s1;
            s3 = s2 * special_s1;
            s4 = s2 * s2;
            s5 = s4 * special_s1;
            s6 = s3 * s3;
            s7 = s4 * s3;

            Densebeta0 << 1.0, special_s1, s2, s3, s4, s5, s6, s7;
            Densebeta1 << 0.0, 1.0, 2.0 * special_s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            Densebeta2 << 0.0, 0.0, 2.0, 6.0 * special_s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            Densebeta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * special_s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            Densebeta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * special_s1, 360.0 * s2, 840 * s3;
            dense_alpha += 1.0 / K / dense_sample_resolution_;

            special_s1 += special_step;

            dense_pos = c.transpose() * Densebeta0;
            dense_vel = c.transpose() * Densebeta1;
            dense_acc = c.transpose() * Densebeta2;
            dense_jer = c.transpose() * Densebeta3;
            dense_sna = c.transpose() * Densebeta4;
            
            dense_omg = (l == 0 || l == disQuantity) ? 0.5 : 1.0;

            gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
            gradv_dense.setZero(); grada_dense.setZero(); gradj_dense.setZero(); gradv.setZero();
            double cost_mm_feasible = 0.0;
            if(feasibilityGradCostCar(dense_vel.head(2), dense_acc.head(2), dense_jer.head(2), trajid,
                                      gradv_2d, grada_2d, gradj_2d,
                                      cost_mm_feasible, true)){
              double cost_dense_log = pen_min_vel_log * cost_mm_feasible;
              
              gradv.head(2)       = -2 * grad_pen_min_vel_log * cost_mm_feasible * vel.head(2);
              gradv_dense.head(2) = pen_min_vel_log * gradv_2d;
              grada_dense.head(2) = pen_min_vel_log * grada_2d;
              gradj_dense.head(2) = pen_min_vel_log * gradj_2d;

              gradViolaVc = beta1 * gradv.transpose();
              gradViolaVt = alpha * gradv.transpose() * acc;
              gradViolaVc_dense = Densebeta1  * gradv_dense.transpose();
              gradViolaVt_dense = dense_alpha * gradv_dense.transpose() * dense_acc;
              gradViolaAc_dense = Densebeta2  * grada_dense.transpose();
              gradViolaAt_dense = dense_alpha * grada_dense.transpose() * dense_jer;
              gradViolaJc_dense = Densebeta3  * gradj_dense.transpose();
              gradViolaJt_dense = dense_alpha * gradj_dense.transpose() * dense_sna;

              SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += dense_omg * special_step * (gradViolaVc + gradViolaVc_dense + gradViolaAc_dense + gradViolaJc_dense);
              gdT(i) += dense_omg * (cost_dense_log / K / dense_sample_resolution_ + special_step * (gradViolaVt + gradViolaVt_dense + gradViolaAt_dense + gradViolaJt_dense));
              costs(3) += dense_omg * special_step * cost_dense_log;
            }
          }
        }
        // joint feasibility
        gradp.setZero(); gradv.setZero(); grada.setZero();
        double cost_joint_feasible = 0.0;
        if(feasibilityGradCostJoint(pos, vel, acc, gradp, gradv, grada, cost_joint_feasible)){
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * (cost_joint_feasible / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          costs(4) += omg * step * cost_joint_feasible;
        }
        s1 += step;
        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
    }
  }

  bool PolyTrajOptimizer::obstacleGradCostforMM(const int i_dp,
                                            const Eigen::VectorXd &pos,
                                            const Eigen::VectorXd &vel,
                                            const int trajid,
                                            Eigen::VectorXd &gradp,
                                            Eigen::VectorXd &gradv,
                                            double &costp,
                                            double &costp_mani,
                                            double &costp_self){
    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;
    bool ret = false;

    gradp.setZero();
    gradv.setZero();
    costp = 0;
    costp_mani = 0;
    costp_self = 0;

    double dist;
    double dist_err;
    double dist_err_2, dist_err_3;
    Eigen::Vector4d dist_grad4 = Eigen::Vector4d::Zero();
    Eigen::VectorXd curr_grad(traj_dim_);
    curr_grad.setZero();
    ros::Time t1, t2, t3, t4;

    std::vector<Eigen::Vector3d> car_pts;
    std::vector<Eigen::Matrix2d> car_dPtsdv_list;
    std::vector<Eigen::Vector2d> car_dPtsdYaw_list;
    Eigen::Vector2d dYawdV = mm_config_->caldYawdV(vel.head(2));
    mm_config_->getCarPtsGradNew(pos.head(2), vel.head(2), singul_container_[trajid], Eigen::Vector3d(0, 0, 0), car_pts, car_dPtsdYaw_list);
    for(unsigned int i = 0; i < car_pts.size(); ++i){
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGrad(car_pts[i], dist, dist_grad);
      dist_err = mobile_base_check_radius_ + safe_margin_ - dist;
      if (dist_err > 0){
        dist_err_2 = dist_err * dist_err;
        dist_err_3 = dist_err_2 * dist_err;
        ret = true;
        costp += wei_obs_ * dist_err_3;
        gradp.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2 * dist_grad.head(2);
        gradv.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2 * dist_grad.head(2).transpose() * car_dPtsdYaw_list[i] * dYawdV;
      }
    }

    if(manipulator_dof_ < 1) return ret;
    Eigen::Matrix4d T_w_q = Eigen::Matrix4d::Identity(), T_w_q_grad_x, T_w_q_grad_y;
    Eigen::Matrix2d R = mm_config_->calR(vel.head(2), singul_container_[trajid]);
    T_w_q.block(0, 0, 2, 2) = R;
    T_w_q.block(0, 3, 2, 1) = pos.head(2);
    T_w_q_grad_x.setZero();
    T_w_q_grad_x(0, 3) = 1;
    T_w_q_grad_y.setZero();
    T_w_q_grad_y(1, 3) = 1;
    // check mani link with obs
    std::vector<Eigen::Matrix4d> T_joint;
    std::vector<Eigen::Matrix4d> T_joint_grad;

    mm_config_->getJointTrans(pos.tail(manipulator_dof_), T_joint, T_joint_grad);

    Eigen::Matrix4d T_now = T_w_q * T_q_0_;
    std::vector<Eigen::Matrix4d> T_grad_list(2 + manipulator_dof_), T_grad_list_self(manipulator_dof_);
    Eigen::Matrix4d T_caldv = T_q_0_; // nouse
    double yaw = atan2(singul_container_[trajid] * vel(1), singul_container_[trajid] * vel(0));
    Eigen::Matrix2d dRdYaw;
    dRdYaw << -sin(yaw), -cos(yaw),
              cos(yaw), -sin(yaw);
    Eigen::Matrix4d dTdYaw;
    dTdYaw.setZero();
    dTdYaw.block(0, 0, 2, 2) = dRdYaw;
    dTdYaw = dTdYaw * T_q_0_;
    T_grad_list[0] = T_w_q_grad_x * T_q_0_;
    T_grad_list[1] = T_w_q_grad_y * T_q_0_;

    Eigen::Matrix4d dRldv;
    Eigen::Vector3d pt_on_link;
    double ground_err = 0.0;
    double wei_ground_arg = 5.0;

    double costp_mani_nume = 0;
    for(int i = 0; i < manipulator_dof_; ++i){
      for(int j = 0; j < i + 2; ++j){
        T_grad_list[j] = T_grad_list[j] * T_joint[i];
      }
      T_caldv = T_caldv * T_joint[i];
      T_grad_list[i + 2] = T_now * T_joint_grad[i];
      T_now = T_now * T_joint[i];
      dTdYaw = dTdYaw * T_joint[i];
      
      // get ESDF value & grad
      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0 / (double)pts_size;
      factor = 1.0;
      for(int j = 0; j < pts_size; ++j){
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateEDTWithGrad(pt_on_link, dist, dist_grad);
        dist_err = manipulator_thickness_ + safe_margin_mani_ - dist;
        if(dist_err > 0){
          ret = true;
          dist_grad4.segment(0, 3) = dist_grad;
          costp_mani += wei_mani_obs_ * factor * pow(dist_err, 3);
          costp_mani_nume += wei_mani_obs_ * factor * pow(dist_err, 3);
          curr_grad.setZero();
          Eigen::Vector4d fac_temp;
          for(int k = 0; k < i + 3; ++k){
            curr_grad(k) -= wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2) * dist_grad4.transpose() * (T_grad_list[k] * manipulator_link_pts_[i].col(j));
          }
          gradp += curr_grad;

          curr_grad.setZero();
          double dDistdYaw = dist_grad4.transpose() * (dTdYaw * manipulator_link_pts_[i].col(j));
          Eigen::Vector2d gradv_temp = wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2) * dDistdYaw * dYawdV;
          curr_grad.head(2) -= gradv_temp;
          gradv += curr_grad;
        }
        Eigen::VectorXd dzdP = Eigen::VectorXd::Zero(4);
        dzdP(2) = 1;
        ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_ + map_resolution_ - pt_on_link(2);
        if(ground_err > 0){
          ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_ - pt_on_link(2);
          double f_temp = 0, df_ground = 0;
          if(smoothedL1(ground_err, 0.0005, f_temp, df_ground)){
            costp_mani += wei_mani_obs_ * wei_ground_arg * factor * f_temp;
            curr_grad.setZero();
            for(int k = 0; k < i + 3; ++k){
              if(ground_err > 0){
                curr_grad(k) -= wei_mani_obs_ * wei_ground_arg * factor * df_ground * dzdP.transpose() * (T_grad_list[k] * manipulator_link_pts_[i].col(j));
              }
            }
            gradp += curr_grad;
          }
        }
      }
    }

    // check mani link with robot
    T_now = T_q_0_ * T_joint[0];
    T_grad_list_self[0] = T_q_0_ * T_joint_grad[0];
    mm_config_->getCarPts(Eigen::Vector3d::Zero(), car_pts, Eigen::Vector3d::Zero());
    double factor2 = 1.0;
    for(int i = 1; i < manipulator_dof_; ++i){ // car will not collide with link 0
      for(int j = 0; j < i; ++j){
        T_grad_list_self[j] = T_grad_list_self[j] * T_joint[i];
      }
      T_grad_list_self[i] = T_now * T_joint_grad[i];
      T_now *= T_joint[i];
      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0;
      double wei_temp = wei_mani_self_ * factor * factor2;
      for(int j = 0; j < pts_size; ++j){
        // with car
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
        for(unsigned int m = 0; m < car_pts.size(); ++m){
          dist = (car_pts[m] - pt_on_link).norm(); // coarse distance
          dist_err = manipulator_thickness_ + mobile_base_check_radius_ + self_safe_margin_ - dist; //refine distance
          if(dist_err > 0){
            dist_err_2 = dist_err * dist_err;
            dist_err_3 = dist_err_2 * dist_err;
            dist_grad4.segment(0, 3) = (pt_on_link - car_pts[m]) / dist;
            ret = true;
            costp_self += wei_temp * dist_err_3;
            curr_grad.setZero();
            for(int k = 0; k < i + 1; ++k){
              curr_grad(mobile_base_dof_ + k) -= 3.0 * wei_temp * dist_err_2 * (T_grad_list_self[k] * manipulator_link_pts_[i].col(j)).transpose() * dist_grad4;
            }
            gradp += curr_grad;
          }
        }

        // manipulator self-collision
        if(i < 2) continue;
        Eigen::Matrix4d T_now_self_mani = T_joint[i];
        std::vector<Eigen::Matrix4d> T_grad_self_mani_list;
        T_grad_self_mani_list.reserve(i);
        T_grad_self_mani_list.push_back(T_joint_grad[i]);
        // FIXME obj on ee may collide with arm i - 1
        for(int m = i - 2; m >= 0; --m){
          for(unsigned int n = 0; n < T_grad_self_mani_list.size(); ++n){
            T_grad_self_mani_list[n] = T_joint[m + 1] * T_grad_self_mani_list[n];
          }
          T_grad_self_mani_list.push_back(T_joint_grad[m + 1] * T_now_self_mani);
          T_now_self_mani = T_joint[m + 1] * T_now_self_mani;
          Eigen::Vector3d pt_on_link_to_check_m, pt_on_link_m;
          pt_on_link_m = (T_now_self_mani * manipulator_link_pts_[i].col(j)).head(3);
          int pts_size_temp = manipulator_link_pts_[m].cols();
          double factor = 1.0 / (double)pts_size_temp;
          for(int n = 0; n < pts_size_temp; ++n){
            pt_on_link_to_check_m = (manipulator_link_pts_[m].col(n)).head(3);
            dist = (pt_on_link_m - pt_on_link_to_check_m).norm(); // coarse distance
            dist_err = 2 * manipulator_thickness_ + self_safe_margin_ - dist; //refine distance
            if(dist_err > 0){
              dist_err_2 = dist_err * dist_err;
              dist_err_3 = dist_err_2 * dist_err;
              dist_grad4.segment(0, 3) = (pt_on_link_m - pt_on_link_to_check_m) / dist;
              ret = true;
              costp_self += wei_mani_self_ * factor * dist_err_3;
              curr_grad.setZero();
              int temp_grad_list_size = T_grad_self_mani_list.size() - 1;
              for(int k = temp_grad_list_size; k >= 0; --k){
                curr_grad(mobile_base_dof_ + m + k + 1) -= 3 * wei_mani_self_ * factor * dist_err_2 * (T_grad_self_mani_list[temp_grad_list_size - k] * manipulator_link_pts_[i].col(j)).transpose() * dist_grad4;
              }
              gradp += curr_grad;
            }
          }
        }
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostCar(const Eigen::Vector2d &vel, const Eigen::Vector2d &acc, const Eigen::Vector2d &jer, const int trajid,
                                                 Eigen::Vector2d &gradv_2d, Eigen::Vector2d &grada_2d, Eigen::Vector2d &gradj_2d,
                                                 double &cost_mm_feasible, bool dense_sample){
    bool ret = false;
    double pen, f, df;
    cost_mm_feasible = 0;

    const double vTv = vel.transpose() * vel;
    const double aTv = acc.transpose() * vel;
    const double aTBv = acc.transpose() * B_h_ * vel;
    const double jTBv = jer.transpose() * B_h_ * vel;
    const double v_norm = vel.norm();
    const double vTv_inv = 1.0 / vTv; // avoid siguality vel = 0
    const double vTv_inv2 = vTv_inv * vTv_inv;

    
    // Check minimum speed
    if(dense_sample){ 
      pen = non_singul_v_ * non_singul_v_ - vel.squaredNorm();
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_mm_feasible += wei_feas_ * 1e6 * f;
        gradv_2d += -wei_feas_ * 1e6 * df * 2.0 * vel;
        ret = true;
      }
    }

    // Linear velocity
    pen = vel.squaredNorm() - max_vel_ * max_vel_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * vel;
      ret = true;
    }

    double omega = aTBv * vTv_inv;
    double wheel_omega_left = (2.0 * singul_container_[trajid] * v_norm - mobile_base_wheel_base_ * omega) / (2.0 * mobile_base_wheel_radius_);
    Eigen::Vector2d dOmegadV = B_h_.transpose() * acc / vTv - (aTBv * vel + vel.transpose() * B_h_.transpose() * acc * vel) / vTv / vTv;
    Eigen::Vector2d dOmegadA = vTv_inv * (B_h_ * vel);
    // Left wheel rotation speed
    pen = wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_left * (2.0 * singul_container_[trajid] / v_norm * vel - mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += -wei_feas_ * df * 2.0 * wheel_omega_left * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotation speed
    double wheel_omega_right = (2.0 * singul_container_[trajid] * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen = wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_right * (2.0 * singul_container_[trajid] / v_norm * vel + mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_omega_right * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
    Eigen::Vector2d dAlphadV = (B_h_.transpose() * jer * vTv - 2.0 * jTBv * vel) * vTv_inv2
                              - 2.0 * (aTBv * acc + aTv * B_h_.transpose() * acc) * vTv_inv2
                              + 8.0 * vTv_inv2 * vTv_inv * aTBv * aTv * vel;
    Eigen::Vector2d dAlphadA = -2.0 * vTv_inv2 * (aTv * B_h_ * vel + aTBv * vel);
    Eigen::Vector2d dAlphadJ = B_h_ * vel * vTv_inv;

    // Left wheel rotational acceleration
    double wheel_alpha_left = (2.0 * singul_container_[trajid] * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv - mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * vel / v_norm - mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += -wei_feas_ * df * 2.0 * wheel_alpha_left * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotational acceleration
    double wheel_alpha_right = (2.0 * singul_container_[trajid] * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv + mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * vel / v_norm + mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJoint(const Eigen::VectorXd &pos, const Eigen::VectorXd &vel, const Eigen::VectorXd &acc, 
                                                    Eigen::VectorXd &gradp, Eigen::VectorXd &gradv, Eigen::VectorXd &grada,
                                                    double &cost_joint_feasible){
    bool ret = false;
    double pen, f, df;

    // pos limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = pos(mobile_base_dof_ + i) - max_joint_pos_[i];
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df;
        ret = true;
      }

      pen = min_joint_pos_[i] - pos(mobile_base_dof_ + i);
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df * (-1.0);
        ret = true;
      }
    }

    // vel limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = vel(mobile_base_dof_ + i) * vel(mobile_base_dof_ + i) - max_joint_vel_ * max_joint_vel_;
      f = 0, df = 0;
      if(smoothedL1(pen,0.005,f,df)){
        gradv(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * vel(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    // acc limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = acc(mobile_base_dof_ + i) * acc(mobile_base_dof_ + i) - max_joint_acc_ * max_joint_acc_;
      f = 0, df = 0;
      if(smoothedL1(pen,0.005,f,df)){
        grada(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * acc(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    return ret;
  }

  int PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           const double start_yaw,
                                           const int _start_singul,
                                           const bool start_gripper,
                                           const double end_yaw,
                                           const bool end_gripper,
                                           const Eigen::Vector2d init_ctrl,
                                           const int continous_failures_count,
                                           std::vector<std::vector<Eigen::VectorXd>> &simple_path_container,  // 前进/倒车为一段, 每段N个点x, y, mani_angle
                                           std::vector<std::vector<double>> &yaw_list_container,
                                           std::vector<poly_traj::MinSnapOpt<8>> &frontendMJ_container,
                                           std::vector<int> &singul_container){
    Eigen::VectorXd start_pos = iniState.col(0);
    Eigen::VectorXd end_pos = finState.col(0);
    Eigen::VectorXd start_vel = iniState.col(1);
    Eigen::VectorXd end_vel = finState.col(1);
    int start_singul = _start_singul;
    std::vector<Eigen::VectorXd> t_list_container; t_list_container.clear();
    vector<Eigen::VectorXd> simple_path; simple_path.clear();
    std::vector<double> yaw_list; yaw_list.clear();
    Eigen::VectorXd t_list;
    
    simple_path_container.clear();
    yaw_list_container.clear();
    frontendMJ_container.clear();
    singul_container.clear();

    /* astar search and get the simple path*/
    int status = kino_a_star_->KinoAstarSearchAndGetSimplePath(start_pos, start_vel, start_yaw, start_singul, start_gripper,
                                                              end_pos, end_vel, end_yaw, end_gripper, init_ctrl,
                                                              continous_failures_count,
                                                              simple_path_container, yaw_list_container, 
                                                              singul_container, t_list_container);
  
    if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){
      return status;
    }
    Eigen::MatrixXd innerPts;
    Eigen::MatrixXd headState, tailState;
    headState.resize(traj_dim_, 4);
    tailState.resize(traj_dim_, 4);
    frontendMJ_container.resize(simple_path_container.size());
    for(unsigned int i = 0; i < simple_path_container.size(); ++i){

      int piece_num = simple_path_container[i].size() - 1;
      if (piece_num > 1){
        innerPts.resize(traj_dim_, piece_num - 1);
        for (int j = 0; j < piece_num - 1; ++j){
          innerPts.col(j) = (simple_path_container[i])[j + 1]; // front end
        }
      }
      else{
        // ROS_WARN("simple path is too short");
        piece_num = 2;
        innerPts.resize(traj_dim_, 1);
        innerPts.col(0) = ((simple_path_container[i])[0] + (simple_path_container[i])[1]) / 2;
        simple_path_container[i].insert(simple_path_container[i].begin() + 1, innerPts.col(0));
        t_list.resize(2);
        t_list.setConstant((t_list_container[i])[0] / 2);
        t_list_container[i] = t_list;
      }

      t_list = t_list_container[i];
      // t_list = t_list * 1.2;
      Eigen::VectorXd angle1, angle2;
      double err;
      // double max_t = t_list[0];
      double t_acc = max_joint_vel_ / max_joint_acc_;
      double dist_acc = max_joint_acc_ * t_acc * t_acc;
      // std::cout << "astarWithMinTraj 8 " << i << "\n";
      for (int j = 0; j < piece_num; ++j){
        angle1 = (simple_path_container[i])[j].tail(manipulator_dof_);
        angle2 = (simple_path_container[i])[j + 1].tail(manipulator_dof_);
        double max_t = t_list[j];
        for(int k = 0; k < manipulator_dof_; ++k){
          if(angle1(k) - angle2(k) >= 0)
            err = angle1(k) - angle2(k);
          else
            err = angle2(k) - angle1(k);
          double t_acc_vel = 0;
          if(j == 0 || j == piece_num - 1){
            if(err <= dist_acc){
              t_acc_vel = sqrt(err / max_joint_acc_);
            }else{
              t_acc_vel = (err - dist_acc) / max_joint_vel_ + 2 * t_acc;
            }
          }else{
            t_acc_vel = err / max_joint_vel_;
          }
          max_t = max(t_acc_vel, max_t);
        }
        t_list[j] = max_t;
      }
      t_list[0] *= 1.5;
      t_list[piece_num - 1] *= 1.5;

      if(i > 0){
        headState.setZero();
        headState.col(0) = (simple_path_container[i])[0];
        headState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
      }else{
        headState = iniState;
        if(_start_singul == 0){
          headState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
        }
      }
      if(i < simple_path_container.size() - 1 || status == KinoAstar::REACH_HORIZON){
        tailState.setZero();
        tailState.col(0) = (simple_path_container[i]).back();
        tailState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i].back()), sin(yaw_list_container[i].back()));
      }else{
        tailState = finState; // end vel acc jer dir is foreward
        tailState.col(1).head(2) = singul_container[i] * tailState.col(1).head(2);
        tailState.col(2).head(2) = singul_container[i] * tailState.col(2).head(2);
        tailState.col(3).head(2) = singul_container[i] * tailState.col(3).head(2);
      }
      frontendMJ_container[i].reset(headState, tailState, piece_num);
      frontendMJ_container[i].generate(innerPts, t_list_container[i]);
    }
    return status;
  }

  void PolyTrajOptimizer::displayFrontEndMesh(std::vector<Eigen::VectorXd> &simple_path_full, vector<double> &yaw_list){
    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::MarkerArray marker_array, marker_array_all;
    visualization_msgs::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);
    
    for(unsigned int i = 0; i < simple_path_full.size(); i+=1){
      car_state.head(2) = simple_path_full[i].head(2);
      car_state(2) = yaw_list[i];
      joint_state = simple_path_full[i].tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_front_end", i, 0.07, car_state, joint_state, true); // TODO gripper state
      marker_array_all.markers.insert(marker_array_all.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
    }

    front_end_mm_mesh_vis_pub_.publish(marker_array_all);

  }

  void PolyTrajOptimizer::clear_resize_Cps_container(int container_size){
    cps_container_.clear();
    cps_container_.resize(container_size);
  }

  void PolyTrajOptimizer::setControlPoints(const int trajid, const Eigen::MatrixXd &points){
    cps_container_[trajid].resize_cp(points.cols());
    cps_container_[trajid].points = points;
  }

}