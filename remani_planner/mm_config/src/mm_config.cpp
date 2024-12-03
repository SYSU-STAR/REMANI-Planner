#include "mm_config/mm_config.hpp"

namespace remani_planner
{

void MMConfig::setParam(ros::NodeHandle &nh, const std::shared_ptr<GridMap>& env){
    grid_map_ = env;
    setParam(nh);
}

void MMConfig::setParam(ros::NodeHandle &nh){
    nh.param("mm/mobile_base_dof", mobile_base_dof_, -1);
    nh.param("mm/mobile_base_length", mobile_base_length_, -1.0);
    nh.param("mm/mobile_base_width", mobile_base_width_, -1.0);
    nh.param("mm/mobile_base_height", mobile_base_height_, -1.0);
    nh.param("mm/mobile_base_check_radius", mobile_base_check_radius_, -1.0);

    nh.param("mm/mobile_base_wheel_base", mobile_base_wheel_base_, -1.0);
    nh.param("mm/mobile_base_wheel_radius", mobile_base_wheel_radius_, -1.0);
    nh.param("mm/mobile_base_max_wheel_omega", mobile_base_max_wheel_omega_, -1.0);
    nh.param("mm/mobile_base_max_wheel_alpha", mobile_base_max_wheel_alpha_, -1.0);
    mobile_base_max_vel_ = mobile_base_max_wheel_omega_ * mobile_base_wheel_radius_; // NOTE the maximum value, max vel depends on current omega
    mobile_base_max_acc_ = mobile_base_max_wheel_alpha_ * mobile_base_wheel_radius_; // NOTE approximate value, max acc depends on current alpha and direction of velocity 

    nh.param("mm/manipulator_dof", manipulator_dof_, -1);
    nh.param("mm/manipulator_thickness", manipulator_thickness_, -1.0);

    nh.param("grid_map/resolution", map_resolution_, 0.05);

    nh.param("optimization/safe_margin", car_safe_margin_, -1.0);
    nh.param("optimization/safe_margin_mani", mani_safe_margin_, -1.0);
    nh.param("optimization/self_safe_margin", self_safe_margin_, -1.0);
    nh.param("optimization/ground_safe_dis", ground_safe_dis_, 0.1);

    manipulator_min_pos_.resize(manipulator_dof_);
    manipulator_max_pos_.resize(manipulator_dof_);
    std::vector<double> pos_limit{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    nh.param<std::vector<double>>("mm/manipulator_min_pos", pos_limit, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    for(int i = 0; i < manipulator_dof_; i++){
        manipulator_min_pos_(i) = pos_limit[i];
    }
    nh.param<std::vector<double>>("mm/manipulator_max_pos", pos_limit, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    for(int i = 0; i < manipulator_dof_; i++){
        manipulator_max_pos_(i) = pos_limit[i];
    }

    std::vector<double> manipulator_config;
    nh.getParam("mm/manipulator_config", manipulator_config);
    manipulator_config_.resize(manipulator_config.size());
    for(int i = 0; i < manipulator_config.size(); i++){
        manipulator_config_(i) = manipulator_config[i];
    }

    std::vector<double> base_mani_fixed_joint_xyz_ypr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    nh.param<std::vector<double>>("mm/base_mani_fixed_joint_xyz_ypr", base_mani_fixed_joint_xyz_ypr, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    T_q_0_ = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond quad = Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[3], Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[4], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(base_mani_fixed_joint_xyz_ypr[5], Eigen::Vector3d::UnitX());
    T_q_0_.block(0, 0, 3, 3) = quad.toRotationMatrix();
    T_q_0_(0, 3) = base_mani_fixed_joint_xyz_ypr[0];
    T_q_0_(1, 3) = base_mani_fixed_joint_xyz_ypr[1];
    T_q_0_(2, 3) = base_mani_fixed_joint_xyz_ypr[2];

    nh.param("mm/use_fast_armer", useFastArmer_, true);

    std::string mesh_path = ros::package::getPath("mm_config")  + "/meshes/";

    mesh_resource_mobile_base_ = "file://" + mesh_path + "mobile_base.STL";

    mesh_resource_fastarmer_base0_ = "file://" + mesh_path + "FastArmer/base_link.STL";
    mesh_resource_fastarmer_link1_ = "file://" + mesh_path + "FastArmer/link1.STL";
    mesh_resource_fastarmer_link2_ = "file://" + mesh_path + "FastArmer/link2.STL";
    mesh_resource_fastarmer_link3_ = "file://" + mesh_path + "FastArmer/link3.STL";
    mesh_resource_fastarmer_link4_ = "file://" + mesh_path + "FastArmer/link4.STL";
    mesh_resource_fastarmer_link5_ = "file://" + mesh_path + "FastArmer/link5.STL";
    mesh_resource_fastarmer_link6_ = "file://" + mesh_path + "FastArmer/link6.STL";
    mesh_resource_gripper_base_    = "file://" + mesh_path + "FastArmer/gripper_base.dae";
    mesh_resource_gripper_left_    = "file://" + mesh_path + "FastArmer/gripper_left.dae";
    mesh_resource_gripper_right_   = "file://" + mesh_path + "FastArmer/gripper_right.dae";

    mesh_resource_ur5_base_     = "file://" + mesh_path + "ur5/base.dae";
    mesh_resource_ur5_shoulder_ = "file://" + mesh_path + "ur5/shoulder.dae";
    mesh_resource_ur5_upperarm_ = "file://" + mesh_path + "ur5/upperarm.dae";
    mesh_resource_ur5_forearm_  = "file://" + mesh_path + "ur5/forearm.dae";
    mesh_resource_ur5_wrist1_   = "file://" + mesh_path + "ur5/wrist1.dae";
    mesh_resource_ur5_wrist2_   = "file://" + mesh_path + "ur5/wrist2.dae";
    mesh_resource_ur5_wrist3_   = "file://" + mesh_path + "ur5/wrist3.dae";

    B_h_ << 0.0, -1.0,
            1.0,  0.0;

    T_q_0_ << 1.0, 0  , 0  , 0.03,
                0  , 1.0, 0  , -0.02,
                0  , 0  , 1.0, mobile_base_height_,
                0  , 0  , 0  , 1.0;

    vis_idx_size_ = 100;

    setColorSet();
    setLinkPoint();
}

void MMConfig::setColorSet(){
    color_set_.clear();
    // red
    Eigen::Vector3d color;
    color << 255, 31, 91;
    color /= 255.0;
    color_set_.push_back(color);
    // green
    color << 0, 205, 108;
    color /= 255.0;
    color_set_.push_back(color);
    // blue
    color << 0, 154, 222;
    color /= 255.0;
    color_set_.push_back(color);
    // yellow
    color << 255, 198, 30;
    color /= 255.0;
    color_set_.push_back(color);
    // grey
    color << 160, 177, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // orange
    color << 234, 96, 22;
    color /= 255.0;
    color_set_.push_back(color);
    // purple
    color << 175, 88, 186;
    color /= 255.0;
    color_set_.push_back(color);
    // brown
    color << 166, 118, 29;
    color /= 255.0;
    color_set_.push_back(color);
}

void MMConfig::getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T, Eigen::Matrix4d &T_grad){
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    double linkLength = manipulator_config_(joint_num);
    T = Eigen::Matrix4d::Identity();
    T_grad = Eigen::Matrix4d::Zero();
    if(useFastArmer_){
        switch(joint_num){
            case 0:{
                T(0, 0) = cosTheta;
                T(0, 2) = sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = 0;
                T(1, 2) = -cosTheta;
                T(2, 1) = 1;
                T(2, 2) = 0;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 2) = cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 2) = sinTheta;
                break;
            }
            case 1:{
                T(0, 0) = cosTheta;
                T(0, 1) = -sinTheta;
                T(0, 3) = -linkLength * cosTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = cosTheta;
                T(1, 3) = -linkLength * sinTheta;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 1) = -cosTheta;
                T_grad(0, 3) = linkLength * sinTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 1) = -sinTheta;
                T_grad(1, 3) = -linkLength * cosTheta;
                break;
            }
            case 2:{
                T(0, 0) = -sinTheta;
                T(0, 2) = cosTheta;
                T(0, 3) = -linkLength * sinTheta;
                T(1, 0) = cosTheta;
                T(1, 1) = 0.0;
                T(1, 2) = sinTheta;
                T(1, 3) = linkLength * cosTheta;
                T(2, 1) = 1.0;
                T(2, 2) = 0.0;

                T_grad(0, 0) = -cosTheta;
                T_grad(0, 2) = -sinTheta;
                T_grad(0, 3) = -linkLength * cosTheta;
                T_grad(1, 0) = -sinTheta;
                T_grad(1, 2) = cosTheta;
                T_grad(1, 3) = -linkLength * sinTheta;
                break;
            }
            case 3:{
                T(0, 0) = cosTheta;
                T(0, 2) = -sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = 0;
                T(1, 2) = cosTheta;
                T(2, 1) = -1;
                T(2, 2) = 0;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 2) = -cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 2) = -sinTheta;
                break;
            }
            case 4:{
                T(0, 0) = sinTheta;
                T(0, 2) = cosTheta;
                T(1, 0) = -cosTheta;
                T(1, 1) = 0;
                T(1, 2) = sinTheta;
                T(2, 1) = -1;
                T(2, 2) = 0;

                T_grad(0, 0) = cosTheta;
                T_grad(0, 2) = -sinTheta;
                T_grad(1, 0) = sinTheta;
                T_grad(1, 2) = cosTheta;
                break;
            }
            case 5:{
                T(0, 0) = cosTheta;
                T(0, 1) = -sinTheta;
                T(1, 0) = sinTheta;
                T(1, 1) = cosTheta;
                T(2, 3) = linkLength;

                T_grad(0, 0) = -sinTheta;
                T_grad(0, 1) = -cosTheta;
                T_grad(1, 0) = cosTheta;
                T_grad(1, 1) = -sinTheta;
                break;
            }
                
            default:{
                ROS_ERROR("err joint_num: %d", joint_num);
                break;
            }
        }
    }else{
        if(joint_num == 0 || joint_num == 3){
            T(0, 0) = cosTheta;
            T(0, 2) = -sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = 0;
            T(1, 2) = cosTheta;
            T(2, 1) = -1;
            T(2, 2) = 0;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 2) = -cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 2) = -sinTheta;
        }else if(joint_num == 4){
            T(0, 0) = cosTheta;
            T(0, 2) = sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = 0;
            T(1, 2) = -cosTheta;
            T(2, 1) = 1;
            T(2, 2) = 0;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 2) = cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 2) = sinTheta;
        }else if(joint_num == 1 || joint_num == 2){
            T(0, 0) = cosTheta;
            T(0, 1) = -sinTheta;
            T(0, 3) = linkLength * cosTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = cosTheta;
            T(1, 3) = linkLength * sinTheta;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 1) = -cosTheta;
            T_grad(0, 3) = -linkLength * sinTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 1) = -sinTheta;
            T_grad(1, 3) = linkLength * cosTheta;
        }else if (joint_num == 5){
            T(0, 0) = cosTheta;
            T(0, 1) = -sinTheta;
            T(1, 0) = sinTheta;
            T(1, 1) = cosTheta;
            T(2, 3) = linkLength;

            T_grad(0, 0) = -sinTheta;
            T_grad(0, 1) = -cosTheta;
            T_grad(1, 0) = cosTheta;
            T_grad(1, 1) = -sinTheta;
        }
    }

    
}

void MMConfig::visMM(ros::Publisher &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    if(pub.getNumSubscribers() < 1) return;
    visualization_msgs::MarkerArray marker_array;
    getMMMarkerArray(marker_array, ns, idx, alpha, car_state, joint_state, gripper_close);
    pub.publish(marker_array);
}

void MMConfig::visMMCheckBall(ros::Publisher &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state){
    if(pub.getNumSubscribers() < 1) return;
    visCarCheckBall(pub, ns, idx, alpha, car_state);
    visManiCheckBall(pub, ns, idx, alpha, car_state, joint_state);
}

void MMConfig::getMMMarkerArray(visualization_msgs::MarkerArray &marker_array, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    marker_array.markers.clear();
    visualization_msgs::MarkerArray car_marker_array = getCarMarkerArray(ns, idx, alpha, car_state);
    marker_array.markers.insert(marker_array.markers.end(), car_marker_array.markers.begin(), car_marker_array.markers.end());;
    visualization_msgs::MarkerArray mani_marker_array = getManiMarkerArray(ns, idx, alpha, car_state, joint_state, gripper_close);
    marker_array.markers.insert(marker_array.markers.end(), mani_marker_array.markers.begin(), mani_marker_array.markers.end());;
}

double MMConfig::calYaw(const Eigen::Vector2d &vel_car, int traj_dir){
    return atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
}

Eigen::Matrix2d MMConfig::calR(const Eigen::Vector2d &vel_car, int traj_dir){
    double yaw = atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
    Eigen::Matrix2d R;
    R << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);
    // R = R * traj_dir / vel_car.norm();
    return R;
}

Eigen::Matrix2d MMConfig::caldRldv(const Eigen::Vector2d &vel_car, const Eigen::Vector2d &delta_p, int traj_dir){
    Eigen::Matrix2d dRldv;
    dRldv << delta_p, B_h_ * delta_p;
    dRldv = dRldv * traj_dir / vel_car.norm() - calR(vel_car, traj_dir) * delta_p * vel_car.transpose() / vel_car.squaredNorm();
    return dRldv;
}

Eigen::Vector2d MMConfig::caldYawdV(const Eigen::Vector2d &vel_car){
    Eigen::Vector2d dYawdV;
    dYawdV = B_h_ * vel_car / vel_car.squaredNorm();
    return dYawdV;
}

void MMConfig::getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts){
    getCarPts(car_state, car_pts, Eigen::Vector3d(0, 0, 0));
}

void MMConfig::getCarPts(const Eigen::Vector3d &car_state, std::vector<Eigen::Vector3d> &car_pts, const Eigen::Vector3d &inflate_size){
    car_pts.clear();
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R;
    R << cos(car_state(2)), -sin(car_state(2)),
         sin(car_state(2)),  cos(car_state(2));
    Eigen::Vector2d corner1 = car_state.head(2) + R * Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner2 = car_state.head(2) + R * Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = car_state.head(2) + R * Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner4 = car_state.head(2) + R * Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();

    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
        }
    }
}

void MMConfig::getCarPtsGrad(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Matrix2d> &dPtdv){
    car_pts.clear();
    dPtdv.clear();
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R = calR(vel_car, traj_dir);
    double v_norm = vel_car.norm();
    double vTv = vel_car.squaredNorm();

    Eigen::Vector2d delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner1 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner1dv;
    dcorner1dv << delta_p, B_h_ * delta_p;
    dcorner1dv = dcorner1dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner2 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner2dv;
    dcorner2dv << delta_p, B_h_ * delta_p;
    dcorner2dv = dcorner2dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner3dv;
    dcorner3dv << delta_p, B_h_ * delta_p;
    dcorner3dv = dcorner3dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner4 = pos_car + R * delta_p;
    Eigen::Matrix2d dcorner4dv;
    dcorner4dv << delta_p, B_h_ * delta_p;
    dcorner4dv = dcorner4dv * traj_dir / v_norm - R * delta_p * vel_car.transpose() / vTv;
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();
    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner1dv);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner2dv);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner3dv);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);
        dPtdv.push_back(dcorner4dv);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm12 * (dcorner2dv - dcorner1dv) + dcorner1dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm23 * (dcorner3dv - dcorner2dv) + dcorner2dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm34 * (dcorner4dv - dcorner3dv) + dcorner3dv);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
            dPtdv.push_back(dl / norm41 * (dcorner1dv - dcorner4dv) + dcorner4dv);
        }
    }
}

void MMConfig::getCarPtsGradNew(const Eigen::Vector2d &pos_car, const Eigen::Vector2d &vel_car, const int traj_dir, const Eigen::Vector3d &inflate_size,
                    std::vector<Eigen::Vector3d> &car_pts, std::vector<Eigen::Vector2d> &dPtdYaw){
    car_pts.clear();
    dPtdYaw.clear();
    double yaw = atan2(traj_dir * vel_car(1), traj_dir * vel_car(0));
    Eigen::Vector3d point_3d;
    Eigen::Matrix2d R, dRdYaw;
    R << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);
    dRdYaw << -sin(yaw), -cos(yaw),
               cos(yaw), -sin(yaw);
    // double v_norm = vel_car.norm();

    Eigen::Vector2d delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner1 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner1dYaw;
    dcorner1dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d( (mobile_base_length_ + inflate_size(0)) / 2 - mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner2 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner2dYaw;
    dcorner2dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_, -(mobile_base_width_ + inflate_size(1)) / 2 + mobile_base_check_radius_);
    Eigen::Vector2d corner3 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner3dYaw;
    dcorner3dYaw = dRdYaw * delta_p;

    delta_p = Eigen::Vector2d(-(mobile_base_length_ + inflate_size(0)) / 2 + mobile_base_check_radius_,  (mobile_base_width_ + inflate_size(1)) / 2 - mobile_base_check_radius_);
    Eigen::Vector2d corner4 = pos_car + R * delta_p;
    Eigen::Vector2d dcorner4dYaw;
    dcorner4dYaw = dRdYaw * delta_p;
    
    double norm12 = (corner2 - corner1).norm();
    double norm23 = (corner3 - corner2).norm();
    double norm34 = (corner4 - corner3).norm();
    double norm41 = (corner1 - corner4).norm();
    
    for(double height = mobile_base_check_radius_; height < mobile_base_height_ + inflate_size(2); height += mobile_base_check_radius_){
        // 四个顶点
        point_3d(2) = height;
        point_3d.head(2) = corner1;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner1dYaw);
        point_3d.head(2) = corner2;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner2dYaw);
        point_3d.head(2) = corner3;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner3dYaw);
        point_3d.head(2) = corner4;
        car_pts.push_back(point_3d);
        dPtdYaw.push_back(dcorner4dYaw);

        for(double dl = mobile_base_check_radius_; dl < norm12; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm12 * (corner2 - corner1) + corner1;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm12 * (dcorner2dYaw - dcorner1dYaw) + dcorner1dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm23; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm23 * (corner3 - corner2) + corner2;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm23 * (dcorner3dYaw - dcorner2dYaw) + dcorner2dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm34; dl+=mobile_base_check_radius_){
            point_3d.head(2) = dl / norm34 * (corner4 - corner3) + corner3;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm34 * (dcorner4dYaw - dcorner3dYaw) + dcorner3dYaw);
        }

        for(double dl = mobile_base_check_radius_; dl < norm41; dl += mobile_base_check_radius_){
            point_3d.head(2) = dl / norm41 * (corner1 - corner4) + corner4;
            car_pts.push_back(point_3d);
            dPtdYaw.push_back(dl / norm41 * (dcorner1dYaw - dcorner4dYaw) + dcorner4dYaw);
        }
    }
}

void MMConfig::CarState2T(const Eigen::Vector3d &car_state, Eigen::Matrix4d &T_car){
    T_car.setIdentity();
    double c = cos(car_state(2)), s = sin(car_state(2));
    Eigen::Matrix3d R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    T_car.block(0, 3, 3, 1) = Eigen::Vector3d(car_state(0), car_state(1), 0.0);
    T_car.block(0, 0, 3, 3) = R;
}

void MMConfig::getJointTMat(const Eigen::VectorXd &theta, std::vector<Eigen::Matrix4d> &T_joint){
    T_joint.clear();
    Eigen::Matrix4d T_temp, T_temp_grad_nouse;
    T_temp = Eigen::Matrix4d::Identity();
    T_joint.push_back(T_temp);
    for(int i = 0; i < manipulator_dof_ - 1; ++i)
    {
        getAJointTran(i, theta(i), T_temp, T_temp_grad_nouse);
        T_joint.push_back(T_temp);
    }
}

bool MMConfig::checkCarObsCollision(Eigen::Vector3d car_state, bool precise, bool safe, double &min_dist){
    std::vector<Eigen::Vector3d> car_pts;
    car_pts.clear();
    getCarPts(car_state, car_pts);
    double dist;
    double safe_dist = safe ? mobile_base_check_radius_ + car_safe_margin_ : mobile_base_check_radius_;
    safe_dist += map_resolution_;
    for(unsigned int i = 0; i < car_pts.size(); ++i){
        if(precise){
            dist = grid_map_->getPreciseDistance(car_pts[i]);
        }else{
            dist = grid_map_->getDistance(car_pts[i]);
        }
        if(dist < safe_dist){
            min_dist = dist;
            return true;
        }
    }
    min_dist = safe_dist;
    return false;
}

bool MMConfig::checkManiObsCollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, double &min_dist){
    Eigen::Matrix4d T_q = Eigen::Matrix4d::Identity();
    T_q(0, 0) = cos(car_state(2));
    T_q(0, 1) = -sin(car_state(2));
    T_q(0, 3) = car_state(0);
    T_q(1, 0) = sin(car_state(2));
    T_q(1, 1) = cos(car_state(2));
    T_q(1, 3) = car_state(1);
    double safe_dist = safe ? manipulator_thickness_ + mani_safe_margin_ : manipulator_thickness_;
    // safe_dist += map_resolution_ / 2.0;
    geometry_msgs::Point pt;
    Eigen::Vector3d pt_on_link;
    Eigen::Matrix4d T_now = T_q * T_q_0_;
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.clear();
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    double dist;
    for(int i = 0; i < manipulator_dof_; ++i){
        T_now = T_now * T_joint[i];
        // get ESDF value
        int pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt.x = pt_on_link(0);
            pt.y = pt_on_link(1);
            pt.z = pt_on_link(2);
            if(pt_on_link(2) < ground_safe_dis_){
                min_dist = pt_on_link(2);
                sphere_occ_.points.push_back(pt);
                return true;
            }
            dist = grid_map_->getPreciseDistance(pt_on_link);
            // dist = grid_map_->getDistance(pt_on_link);
            if(dist < safe_dist){
                sphere_occ_.points.push_back(pt);
                min_dist = dist;
                return true;
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

bool MMConfig::checkCarManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist){
    double safe_dist = safe ? mobile_base_check_radius_ + manipulator_thickness_ + self_safe_margin_ : mobile_base_check_radius_ + manipulator_thickness_;
    std::vector<Eigen::Vector3d> car_pts;
    getCarPts(Eigen::Vector3d::Zero(), car_pts);
    Eigen::Vector3d pt_on_link;
    std::vector<Eigen::Vector3d> pt_to_check_list;
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.clear();
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    int car_pts_size = car_pts.size();
    int pts_size;
    Eigen::Matrix4d T_now = T_q_0_ * T_joint[0];
    for(int i = 1; i < manipulator_dof_; ++i){
        T_now = T_now * T_joint[i];
        pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            for(int k = 0; k < car_pts_size; ++k){
                if((pt_on_link - car_pts[k]).norm() < safe_dist){
                    min_dist = (pt_on_link - car_pts[k]).norm();
                    return true;
                }
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

bool MMConfig::checkManiManiCollision(Eigen::VectorXd mani_state, bool safe, double &min_dist){
    double safe_dist = safe ? 2.0 * manipulator_thickness_ + self_safe_margin_ : 2.0 * manipulator_thickness_;
    Eigen::Vector3d pt_on_link;
    std::vector<Eigen::Vector3d> pt_to_check_list;
    pt_to_check_list.reserve(20);
    std::vector<Eigen::Matrix4d> T_joint, T_joint_grad_nouse;
    T_joint.reserve(manipulator_dof_);
    T_joint_grad_nouse.reserve(manipulator_dof_);
    getJointTrans(mani_state, T_joint, T_joint_grad_nouse);
    int num_to_check = 0;
    int pts_size;
    Eigen::Matrix4d T_now = Eigen::Matrix4d::Identity();
    for(int i = 0; i < manipulator_dof_; ++i){
        T_now *= T_joint[i];
        if(i >= 2)
            num_to_check += manipulator_link_pts_[i - 2].cols();
        // if(i == manipulator_dof_ - 1)
        //     num_to_check += manipulator_link_pts_[i - 1].cols();
        pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt_to_check_list.push_back(pt_on_link);
            for(int k = 0; k < num_to_check; ++k){
                if((pt_on_link - pt_to_check_list[k]).norm() < safe_dist){
                    // printf("(%d, %d, %d)\n", i, j, k);
                    min_dist = (pt_on_link - pt_to_check_list[k]).norm();
                    return true;
                }
            }
        }
    }
    min_dist = safe_dist;
    return false;
}

bool MMConfig::checkManicollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe){
    double min_dist;
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    return false;
}

bool MMConfig::checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe, int &coll_type /*0: car, 1: mani, 2: car-mani, 3: mani-mani*/){
    double min_dist;
    if(checkCarObsCollision(car_state, true, safe, min_dist)){
        coll_type = 0;
        return true;
    }
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        coll_type = 1;
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        coll_type = 2;
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        coll_type = 3;
        return true;
    }
    coll_type = -1;
    return false;
}

bool MMConfig::checkcollision(Eigen::Vector3d car_state, Eigen::VectorXd mani_state, bool safe){
    double min_dist;
    if(checkCarObsCollision(car_state, true, safe, min_dist)){
        return true;
    }
    if(checkManiObsCollision(car_state, mani_state, safe, min_dist)){
        return true;
    }
    if(checkCarManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    if(checkManiManiCollision(mani_state, safe, min_dist)){
        return true;
    }
    return false;
}

void MMConfig::setLinkPoint()
{
    manipulator_link_pts_.clear();
    Eigen::Matrix4Xd link_pts;
    if(useFastArmer_){
        for(int i = 0; i < manipulator_dof_; ++i){
            switch(i){
            case 0:{
                link_pts.resize(4, 4);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0, 0.05, 1);
                link_pts.col(2) = Eigen::Vector4d(0, 0, -0.05, 1);
                link_pts.col(3) = Eigen::Vector4d(0, -0.05, 0, 1);
                break;
            }
            case 1:{
                link_pts.resize(4, 6);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0.0, 1);
                link_pts.col(1) = Eigen::Vector4d(0.07, 0, 0.0, 1);
                link_pts.col(2) = Eigen::Vector4d(0.14, 0, 0.0, 1);
                link_pts.col(3) = Eigen::Vector4d(0.21, 0, 0.0, 1);
                link_pts.col(4) = Eigen::Vector4d(0.28, 0, 0.0, 1);
                link_pts.col(5) = Eigen::Vector4d(0.35, 0, 0.0, 1);
                break;
            }
            case 2:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 3:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0.0, 0.07, 0.0, 1);
                link_pts.col(1) = Eigen::Vector4d(0.0, 0.14, 0.0, 1);
                link_pts.col(2) = Eigen::Vector4d(0.0, 0.21, 0.0, 1);
                link_pts.col(3) = Eigen::Vector4d(0.0, 0.28, 0.0, 1);
                link_pts.col(4) = Eigen::Vector4d(0.0, 0.35, 0.0, 1);
                // link_pts.col(4) = Eigen::Vector4d(0, 0.4, 0.0, 1);
                break;
            }
            case 4:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 5:{
                link_pts.resize(4, 7);
                link_pts.col(0) = Eigen::Vector4d(0, 0, -0.10, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.03, -0.10, 1);
                link_pts.col(2) = Eigen::Vector4d(0, -0.03, -0.10, 1);
                link_pts.col(3) = Eigen::Vector4d(0, 0.05, -0.05, 1);
                link_pts.col(4) = Eigen::Vector4d(0, -0.05, -0.05, 1);
                link_pts.col(5) = Eigen::Vector4d(0, 0.06, -0.00, 1);
                link_pts.col(6) = Eigen::Vector4d(0, -0.06, -0.00, 1);
                break;
            }
            default:
                break;
            }
            manipulator_link_pts_.push_back(link_pts);
        }
    }else{
        for(int i = 0; i < manipulator_dof_; ++i){
            switch(i){
            case 0:{
                link_pts.resize(4, 2);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.05, 0, 1);
                break;
            }
            case 1:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0.14, 1);
                link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0.14, 1);
                link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0.14, 1);
                link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0.14, 1);
                link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0.14, 1);
                break;
            }
            case 2:{
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0, 1);
                link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0, 1);
                link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0, 1);
                link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0, 1);
                break;
            }
            case 3:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 4:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 5:{
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            default:
                break;
            }
            manipulator_link_pts_.push_back(link_pts);
        }
    }
}

void MMConfig::setGripperPoint(const bool gripper_close){
    if(!useFastArmer_) return;
    if(gripper_close){
        manipulator_link_pts_[5].col(5) = Eigen::Vector4d(0, 0.02, -0.01, 1);
        manipulator_link_pts_[5].col(6) = Eigen::Vector4d(0, -0.02, -0.01, 1);
    }else{
        manipulator_link_pts_[5].col(5) = Eigen::Vector4d(0, 0.06, -0.00, 1);
        manipulator_link_pts_[5].col(6) = Eigen::Vector4d(0, -0.06, -0.00, 1);
    }
}

void MMConfig::visCarCheckBall(ros::Publisher &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &state){
    std::vector<Eigen::Vector3d> car_pts;
    getCarPts(state, car_pts, Eigen::Vector3d::Zero());

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = idx;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.r = color_set_[0](0);
    sphere.color.g = color_set_[0](1);
    sphere.color.b = color_set_[0](2);
    sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
    sphere.scale.x = mobile_base_check_radius_ * 2.0;
    sphere.scale.y = mobile_base_check_radius_ * 2.0;
    sphere.scale.z = mobile_base_check_radius_ * 2.0;
    geometry_msgs::Point pt;
    for (unsigned int i = 0; i < car_pts.size(); i++){
      pt.x = car_pts[i](0);
      pt.y = car_pts[i](1);
      pt.z = car_pts[i](2);
      sphere.points.push_back(pt);
    }
    pub.publish(sphere);
}

void MMConfig::visManiCheckBall(ros::Publisher &pub, std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state){
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = idx + 1000;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 0;
    sphere.color.b = 0;
    sphere.color.a = alpha > 1e-5 ? alpha : 1.0;
    sphere.scale.x = manipulator_thickness_ * 2.0;
    sphere.scale.y = manipulator_thickness_ * 2.0;
    sphere.scale.z = manipulator_thickness_ * 2.0;
    geometry_msgs::Point pt;
    
    Eigen::Matrix4d T_q;
    T_q << cos(car_state(2)), -sin(car_state(2)), 0.0, car_state(0),
            sin(car_state(2)),  cos(car_state(2)), 0.0, car_state(1),
            0.0              , 0.0               , 1.0, 0.0,
            0.0              , 0.0               , 0.0, 1.0;
    Eigen::Matrix4d T_now = T_q * T_q_0_;
    Eigen::Vector3d pt_on_link;

    for(int i = 0; i < manipulator_dof_; ++i){
        sphere.color.r = color_set_[i + 1](0);
        sphere.color.g = color_set_[i + 1](1);
        sphere.color.b = color_set_[i + 1](2);
        ++sphere.id;
        Eigen::Matrix4d temp, temp_grad;
        getAJointTran(i, joint_state(i), temp, temp_grad);
        T_now = T_now * temp;
        int pts_size = manipulator_link_pts_[i].cols();
        for(int j = 0; j < pts_size; ++j){
            pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
            pt.x = pt_on_link(0);
            pt.y = pt_on_link(1);
            pt.z = pt_on_link(2);
            sphere.points.push_back(pt);
        }
        pub.publish(sphere);
        sphere.points.clear();
    }
    
}

visualization_msgs::MarkerArray MMConfig::getCarMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &state){
    Eigen::Matrix2d R;
    visualization_msgs::MarkerArray marker_array;

    R << cos(state(2)), -sin(state(2)),
            sin(state(2)),  cos(state(2));
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    pos.head(2) = state.head(2);
    pos(2) = 0.06;
    T.block(0, 0, 2, 2) = R;
    T.block(0, 3, 3, 1) = pos;
    marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 0, ns, alpha, T, mesh_resource_mobile_base_));
    
    return marker_array;
}

visualization_msgs::MarkerArray MMConfig::getManiMarkerArray(std::string ns, int idx, double alpha, const Eigen::Vector3d &car_state, const Eigen::VectorXd &joint_state, const bool &gripper_close){
    Eigen::VectorXd theta = joint_state;
    visualization_msgs::MarkerArray marker_array;

    Eigen::Matrix4d T_q;
    T_q << cos(car_state(2)), -sin(car_state(2)), 0.0, car_state(0),
            sin(car_state(2)),  cos(car_state(2)), 0.0, car_state(1),
            0.0              , 0.0               , 1.0, 0.0,
            0.0              , 0.0               , 0.0, 1.0;

    Eigen::Matrix4d T_now = T_q * T_q_0_;

    if(useFastArmer_){
        Eigen::Matrix4d T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 11, ns, alpha, T_now, mesh_resource_fastarmer_base0_));

        
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_temp(2, 3) = manipulator_config_(0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(0));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 12, ns, alpha, T_now, mesh_resource_fastarmer_link1_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI_2, 0, 0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(1));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 13, ns, alpha, T_now, mesh_resource_fastarmer_link2_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, -M_PI_2);
        T_temp(0, 3) = manipulator_config_(1);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(2));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 14, ns, alpha, T_now, mesh_resource_fastarmer_link3_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI_2, 0, -M_PI);
        T_temp(0, 3) = 0.0650000000000004;
        T_temp(1, 3) = -manipulator_config_(3);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(3));
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 15, ns, alpha, T_now, mesh_resource_fastarmer_link4_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI_2, 0, M_PI_2);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(4)); // theta(4)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 16, ns, alpha, T_now, mesh_resource_fastarmer_link5_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI_2, 0, 0);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(5)); // theta(5)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 17, ns, alpha, T_now, mesh_resource_fastarmer_link6_));
        
        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << 0.005, 0.005, 0.061;
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 18, ns, alpha, T_now, mesh_resource_gripper_base_));

        Eigen::Matrix4d T_gripper = T_now;

        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << -0.003, 0.035, 0.061;
        T_now = T_gripper * T_temp;

        if(gripper_close){
            T_temp.setZero();
            T_temp(3, 3) = 1.0;
            T_temp.block(0, 0, 3, 3) = euler2rotation(M_PI / 6, 0, 0); // theta(4)
            T_now = T_now * T_temp;
        }
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 19, ns, alpha, T_now, mesh_resource_gripper_left_));

        T_temp.setIdentity();
        T_temp.block(0, 3, 3, 1) << -0.003, -0.035, 0.061;
        T_now = T_gripper * T_temp;

        if(gripper_close){
            T_temp.setZero();
            T_temp(3, 3) = 1.0;
            T_temp.block(0, 0, 3, 3) = euler2rotation(-M_PI / 6, 0, 0); // theta(4)
            T_now = T_now * T_temp;
        }
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 20, ns, alpha, T_now, mesh_resource_gripper_right_));
        
    }else{
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 11, ns, alpha, T_now, mesh_resource_ur5_base_));

        Eigen::Matrix4d T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(0));
        T_temp(2, 3) = manipulator_config_(0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 12, ns, alpha, T_now, mesh_resource_ur5_shoulder_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, M_PI_2, 0);
        T_temp(1, 3) = 0.138;
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(1), 0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 13, ns, alpha, T_now, mesh_resource_ur5_upperarm_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(1, 3) = -0.131;
        T_temp(2, 3) = manipulator_config_(1);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(2), 0);
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 14, ns, alpha, T_now, mesh_resource_ur5_forearm_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, M_PI_2, 0);
        T_temp(2, 3) = manipulator_config_(2);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(3), 0); // theta(3)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 15, ns, alpha, T_now, mesh_resource_ur5_wrist1_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(1, 3) = manipulator_config_(3);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, theta(4)); // theta(4)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 16, ns, alpha, T_now, mesh_resource_ur5_wrist2_));

        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, 0, 0);
        T_temp(2, 3) = manipulator_config_(4);
        T_now = T_now * T_temp;
        T_temp.setZero();
        T_temp(3, 3) = 1.0;
        T_temp.block(0, 0, 3, 3) = euler2rotation(0, theta(5), 0); // theta(5)
        T_now = T_now * T_temp;
        marker_array.markers.push_back(getMarker(idx * vis_idx_size_ + 17, ns, alpha, T_now, mesh_resource_ur5_wrist3_));
    }
    

    return marker_array;
}

void MMConfig::visMesh(ros::Publisher &pub, int id, std::string ns, double alpha, Eigen::Vector3d color_rgb, const Eigen::Matrix4d &T, const std::string &mesh_file){
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::Marker meshMarker;
    meshMarker.header.frame_id = "world";
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = true;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    if(id == 2 || id == 102){
        meshMarker.scale.x = 0.001;
        meshMarker.scale.y = 0.001;
        meshMarker.scale.z = 0.001;
    }
    // meshMarker.color.r = color_rgb(0);
    // meshMarker.color.g = color_rgb(1);
    // meshMarker.color.r = color_rgb(2);
    if(alpha >= 0.0)
        meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    pub.publish(meshMarker);
}

visualization_msgs::Marker MMConfig::getMarker(int id, std::string ns, double alpha, const Eigen::Matrix4d &T, const std::string &mesh_file){
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::Marker meshMarker;
    meshMarker.header.frame_id = "world";
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = true;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    if(alpha >= 0.0)
        meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    return meshMarker;
}

}