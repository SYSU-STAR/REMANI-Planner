#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include "traj_utils/plan_container.hpp"
using std::vector;
namespace remani_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_traj_pub;
    ros::Publisher init_ctrl_pts_pub;
    ros::Publisher optmizing_traj_pub;
    ros::Publisher init_waypoints_pub;
    ros::Publisher optimal_ctrl_pts_pub;
    ros::Publisher optimal_waypoints_pub;
    ros::Publisher failed_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher init_list_debug_pub;

    ros::Publisher intermediate_pt0_pub;
    ros::Publisher intermediate_pt1_pub;
    ros::Publisher intermediate_grad0_pub;
    ros::Publisher intermediate_grad1_pub;
    ros::Publisher intermediate_grad_smoo_pub;
    ros::Publisher intermediate_grad_dist_pub;
    ros::Publisher intermediate_grad_feas_pub;

    bool start_visual_;
    
    ros::Subscriber mm_car_odom_sub_;

    ros::Timer benchmark_recorder;

    std::ofstream odom_csv;
    ros::Time t_init;
    ros::Time t_record;

  public:

    PlanningVisualization(/* args */) {}

    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector2d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalTraj(vector<Eigen::Vector2d> global_pts, const double scale, int id);
    void displayInitCtrlPts(vector<Eigen::Vector2d> init_pts, const double scale, int id);
    void displayInitWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id);
    void displayOptimizingTraj(vector<vector<Eigen::Vector2d>> init_trajs, const double scale);
    void displayOptimalCtrlPts(std::vector<Eigen::MatrixXd> optimal_pts, int id);
    void displayOptWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector2d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayInitPathListDebug(vector<Eigen::Vector2d> init_pts, const double scale, int id);

    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
  };
} // namespace remani_planner
#endif